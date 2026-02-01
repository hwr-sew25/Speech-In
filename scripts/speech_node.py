#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import queue
import sounddevice as sd
import vosk
import rospy
import time
import re
import numpy as np 

from std_msgs.msg import String, Header
from speech_in.msg import SpeechCommand, SpeechStatus

class SpeechNode:
    def __init__(self):
        rospy.init_node('speech_node', anonymous=True)
        
        self.samplerate = 16000
        self.device_index = self.find_device_index() 
        
        self.pub_command = rospy.Publisher('/speech_command', SpeechCommand, queue_size=10)
        self.pub_status = rospy.Publisher('/speech_output', SpeechStatus, queue_size=10)
        rospy.Subscriber('/language', String, self.language_callback)
        
        self.q = queue.Queue()
        self.active = False
        self.start_listening_time = 0
        self.TIMEOUT_SEC = 8.0 
        
        self.locations_file = "/home/ubuntu/catkin_ws/src/speech_in/data/locations.csv"
        self.locations = self.load_locations(self.locations_file)

        model_path_de = "/home/ubuntu/catkin_ws/src/speech_in/models/model_de"
        model_path_en = "/home/ubuntu/catkin_ws/src/speech_in/models/model_en"

        if not os.path.exists(model_path_de) or not os.path.exists(model_path_en):
            rospy.logerr("Modelle fehlen! Bitte Pfade prüfen.")
            sys.exit(1)
            
        # VOKABULAR
        vocabulary = [
            "eins", "zwei", "drei", "vier", "fünf", "sechs", "sieben", "acht", "neun", "zehn", 
            "elf", "zwölf", "null", "hundert", "und", "punkt", "komma",
            "erste", "zweite", "dritte", "vierte", "fünfte",
            "zwanzig", "dreißig", "vierzig", "fünfzig", 
            "raum", "wo", "ist", "finde", "suche", "navigiere", "bringe", "mich", "zu",
            "wc", "toilette", "klo", "herren", "damen", "nummer", "etage", "stock", "ebene",
            "im", "in", "am", "an", "dem", "den", "der", "die", "das", "zum", "zur", "neben", "auf",
            "ich", "du", "wir", "möchte", "will", "muss", "kann", "darf", "bitte", "soll", "habe",
            "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten",
            "eleven", "twelve", "zero", "hundred", "first", "second", "third", "fourth", "fifth",
            "twenty", "thirty", "forty", "fifty", "point", "dot",
            "floor", "room", "where", "is", "find", "search", "navigate", "go", "to",
            "bathroom", "restroom", "toilet", "ladies", "mens", "number", "level", 
            "at", "the", "please", "i", "want", "need", "can",
            "[unk]"
        ]
        
        vocab_json = json.dumps(vocabulary)
        rospy.loginfo(f"Lade Modelle... ({len(vocabulary)} Wörter)")

        self.model_de = vosk.Model(model_path_de)
        self.model_en = vosk.Model(model_path_en)

        self.rec_de = vosk.KaldiRecognizer(self.model_de, self.samplerate, vocab_json)
        self.rec_en = vosk.KaldiRecognizer(self.model_en, self.samplerate, vocab_json)
        
        rospy.loginfo("Speech-In BEREIT! (Float-Match Fix)")

    def find_device_index(self):
        devices = sd.query_devices()
        for i, dev in enumerate(devices):
            name = dev['name'].lower()
            if dev['max_input_channels'] > 0:
                if 'respeaker' in name or ('usb' in name and 'mic' in name):
                    rospy.loginfo(f"Auto-Detect: ReSpeaker/USB Mic gefunden auf Index {i} ({dev['name']})")
                    return i
        try: return sd.default.device[0]
        except: return None

    def load_locations(self, filepath):
        locs = {}
        if not os.path.exists(filepath): return locs
        with open(filepath, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) >= 3:
                    key = parts[0].strip().lower().replace("dritte", "3").replace("vierte", "4")
                    room_id = parts[1].strip()
                    try: floor = int(parts[2].strip())
                    except: floor = 0
                    locs[key] = {"target": room_id, "floor": floor}
        return locs

    def text2int(self, text):
        t = text.lower()
        
        # Hundert-Logik
        single_digits = ["eins", "ein", "zwei", "zwo", "drei", "vier", "fünf", "sechs", "sieben", "acht", "neun"]
        for digit in single_digits:
            if f"{digit} hundert" in t:
                t = t.replace(f"{digit} hundert", f"{digit}")
        t = t.replace("hundert", "1")
        
        mapping = {
            "eins": "1", "ein": "1", "eine": "1", "zwei": "2", "zwo": "2",
            "drei": "3", "vier": "4", "fünf": "5", "sechs": "6", "sieben": "7",
            "acht": "8", "neun": "9", "null": "0", "zehn": "10", "elf": "11", "zwölf": "12",
            "und": "", "punkt": ".", "komma": ".", 
            "zwanzig": "20", "dreißig": "30", "vierzig": "40", "fünfzig": "50",
            "erste": "1", "ersten": "1", "zweite": "2", "zweiten": "2", 
            "dritte": "3", "dritten": "3", "vierte": "4", "vierten": "4",
            "fünfte": "5", "fünften": "5", "etage": "", "stock": "", "ebene": "",
            "im": "", "in": "", "am": "", "an": "", "dem": "", "den": "", 
            "der": "", "die": "", "das": "", "zum": "", "zur": "", "neben": "", "auf": "",
            "ich": "", "du": "", "wir": "", "möchte": "", "will": "", "muss": "", 
            "kann": "", "darf": "", "bitte": "", "soll": "", "habe": "", "is": "",
            "nummer": "", "number": "", "nur": "", "no": "", "search": "",
            "toilette": "wc", "klo": "wc", "bedürfnisanstalt": "wc", "sanitärraum": "wc", 
            "waschraum": "wc", "toilet": "wc", "bathroom": "wc", "restroom": "wc", 
            "ladies": "wc", "mens": "wc",
            "one": "1", "two": "2", "three": "3", "four": "4", "five": "5",
            "six": "6", "seven": "7", "eight": "8", "nine": "9", "zero": "0",
            "ten": "10", "eleven": "11", "twelve": "12",
            "twenty": "20", "thirty": "30", "forty": "40", "fifty": "50",
            "point": ".", "dot": ".", 
            "first": "1", "second": "2", "third": "3", "fourth": "4", "fifth": "5",
            "floor": "", "at": "", "the": "", "please": "", "i": "", "want": "", "need": "", "can": ""
        }
        
        words = t.split()
        res = []
        for w in words:
            if w in mapping:
                val = mapping[w]
                if val != "": res.append(val)
            else:
                res.append(w)
                
        clean_str = " ".join(res)
        
        clean_str = re.sub(r'\b([1-9])\s+10\b', r'1\1', clean_str) 
        clean_str = re.sub(r'\b(\d+)\s+(\d+)\b', r'\1.\2', clean_str) 
        clean_str = clean_str.replace("..", ".")
        clean_str = clean_str.replace(".0.", ".0") 
        
        # FIX: Schwebende Punkte entfernen (für "2 . 3.9")
        clean_str = re.sub(r'\s*\.\s*', '.', clean_str)
        
        return clean_str

    def language_callback(self, msg):
        if "off" in str(msg.data).lower():
            self.active = False
            rospy.loginfo("System Paused") 
        else:
            self.active = True
            self.start_listening_time = time.time() + 0.5
            rospy.loginfo("Listening...") 
            with self.q.mutex: self.q.queue.clear()

    def send_status(self, level, text):
        msg = SpeechStatus()
        msg.header = Header(stamp=rospy.Time.now())
        msg.level = level
        msg.message = text
        self.pub_status.publish(msg)

    def audio_callback(self, indata, frames, time, status):
        if indata.shape[1] > 1:
            mono_data = indata[:, 0]
        else:
            mono_data = indata
        self.q.put(bytes(mono_data))

    def process_text(self, text, lang_code):
        text_clean = self.text2int(text)
        
        valid = ["raum", "wc", "room", "1", "2", "3", "4", "5", "0"]
        if not any(x in text_clean for x in valid): return False

        rospy.loginfo(f"[{lang_code}] Input: '{text}' -> Clean: '{text_clean}'")
        
        # 1. Keys
        sorted_keys = sorted(self.locations.keys(), key=len, reverse=True)
        for key in sorted_keys:
            if key in text_clean:
                val = self.locations[key]
                rospy.loginfo(f"-> ZIEL GEFUNDEN: {val['target']} (Match: {key})")
                self.publish_goal(val['target'], val['floor'])
                msg = f"Fahre zu {val['target']}" if lang_code == "DE" else f"Going to {val['target']}"
                if "wc" in key: msg = f"WC {val['target']} gefunden."
                self.send_status("INFO", msg)
                self.active = False 
                return True
            
        detected_match = re.search(r'\b(\d+\.?\d+)\b', text_clean)
        detected_num = detected_match.group(1) if detected_match else None

        # 2. IDs
        for key, val in self.locations.items():
            target = val['target'].lower()
            
            if target in text_clean:
                rospy.loginfo(f"-> ID DIREKT: {val['target']}")
                self.publish_goal(val['target'], val['floor'])
                self.send_status("INFO", f"Fahre zu {val['target']}")
                self.active = False
                return True
            
            if detected_num:
                # String Match
                if detected_num == target:
                     rospy.loginfo(f"-> ID MATCH: {val['target']}")
                     self.publish_goal(val['target'], val['floor'])
                     self.send_status("INFO", f"Fahre zu {val['target']}")
                     self.active = False
                     return True

                # Smart 0 Match
                if detected_num + "0" == target:
                    rospy.loginfo(f"-> ID SMART (0 end): {val['target']}")
                    self.publish_goal(val['target'], val['floor'])
                    self.send_status("INFO", f"Fahre zu {val['target']}")
                    self.active = False
                    return True
                
                # Smart Middle 0
                parts = detected_num.split('.')
                if len(parts) == 2 and len(parts[1]) == 1:
                    if f"{parts[0]}.0{parts[1]}" == target:
                        rospy.loginfo(f"-> ID SMART (0 mid): {val['target']}")
                        self.publish_goal(val['target'], val['floor'])
                        self.send_status("INFO", f"Fahre zu {val['target']}")
                        self.active = False
                        return True
                        
                # FIX: Float Match (1.40 == 1.4)
                try:
                    if float(detected_num) == float(target):
                        rospy.loginfo(f"-> ID FLOAT MATCH: {val['target']}")
                        self.publish_goal(val['target'], val['floor'])
                        self.send_status("INFO", f"Fahre zu {val['target']}")
                        self.active = False
                        return True
                except: pass

        if detected_num:
            if "raum" in text_clean or "room" in text_clean or "wc" in text_clean:
                err = f"Raum {detected_num} nicht gefunden." if lang_code == "DE" else f"Room {detected_num} not found."
                rospy.logwarn(err)
                self.send_status("ERROR", err)
                self.active = False
                return True

        return False

    def publish_goal(self, target, floor):
        cmd = SpeechCommand()
        cmd.header = Header(stamp=rospy.Time.now())
        cmd.command = "navigate"
        cmd.target = target
        cmd.floor = floor
        self.pub_command.publish(cmd)

    def run(self):
        if self.device_index is None: return
        try:
            with sd.InputStream(samplerate=self.samplerate, device=self.device_index, 
                                channels=2, dtype='int16', callback=self.audio_callback,
                                latency='high'):
                rospy.loginfo(f"Audio Stream läuft auf Device {self.device_index} (2 Kanäle).")
                while not rospy.is_shutdown():
                    if not self.active:
                        time.sleep(0.1)
                        with self.q.mutex: self.q.queue.clear()
                        continue
                    if (time.time() - self.start_listening_time) > self.TIMEOUT_SEC:
                        self.send_status("ERROR", "Keine Eingabe (Timeout).")
                        self.active = False
                        continue
                    if self.q.empty():
                        time.sleep(0.01)
                        continue
                    data = self.q.get()
                    if self.rec_de.AcceptWaveform(data):
                        res = json.loads(self.rec_de.Result())
                        txt = res.get("text", "")
                        if txt and self.process_text(txt, "DE"):
                            with self.q.mutex: self.q.queue.clear()
                            continue
                    if self.rec_en.AcceptWaveform(data):
                        res = json.loads(self.rec_en.Result())
                        txt = res.get("text", "")
                        if txt and self.process_text(txt, "EN"):
                            with self.q.mutex: self.q.queue.clear()
                            continue
        except Exception as e:
            rospy.logerr(f"CRITICAL ERROR: {e}")

if __name__ == '__main__':
    try:
        SpeechNode().run()
    except rospy.ROSInterruptException:
        pass
