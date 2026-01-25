#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import subprocess
import rospy
import time
import csv

# --- WICHTIG: Pfad zu den generierten Messages ---
# Fügt den Pfad hinzu, damit Python das Modul 'speech_in' findet
sys.path.append("/home/ubuntu/catkin_ws/src/Messages/generated_msgs")

from std_msgs.msg import String  # Brauchen wir ggf. nicht mehr, aber stört nicht
from speech_in.msg import SpeechCommand, SpeechStatus  # <-- NEUE IMPORTS

from vosk import Model, KaldiRecognizer

# --- KONFIGURATION ---
MODEL_PATH_DE = "/home/ubuntu/catkin_ws/src/speech_in/models/model_de"
MODEL_PATH_EN = "/home/ubuntu/catkin_ws/src/speech_in/models/model_en"
CSV_PATH = "/home/ubuntu/catkin_ws/src/speech_in/config/rooms.csv"
ALSA_DEVICE = "plughw:1,0" 
TIMEOUT_SECONDS = 7 

# --- WORTSCHATZ ---
TRIGGERS_DE = ["raum", "zimmer", "büro", "saal", "labor", "werkstatt", "wc", "toilette", "klo", "bad", "mensa", "kantine"]
TRIGGERS_EN = ["room", "office", "hall", "lab", "chamber", "restroom", "bathroom", "toilet", "canteen", "cafeteria"]

CATEGORY_MAPPING = {
    "wc": "WC", "toilette": "WC", "klo": "WC", "bad": "WC", 
    "mensa": "Mensa", "kantine": "Mensa",
    "restroom": "WC", "bathroom": "WC", "toilet": "WC"
}

FLOOR_MAPPING = {
    "erdgeschoss": "EG", "unten": "EG", "eingang": "EG", 
    "ersten": "1", "eins": "1", "erste": "1",
    "zweiten": "2", "zwei": "2", "zweite": "2",
    "dritten": "3", "drei": "3", "dritte": "3",
    "vierten": "4", "vier": "4", "vierte": "4",
    "fünften": "5", "fünf": "5", "fünfte": "5",
    "ground": "EG", "first": "1", "one": "1", "second": "2", "two": "2", "third": "3", "three": "3", "fourth": "4", "four": "4"
}

# --- PARSER ---
def german_word_to_int(word):
    units = {'eins':1,'ein':1,'zwei':2,'zwo':2,'drei':3,'vier':4,'fünf':5,'sechs':6,'sieben':7,'acht':8,'neun':9,'zehn':10,'elf':11,'zwölf':12,'sechzehn':16,'siebzehn':17}
    tens = {'zwanzig':20,'dreißig':30,'vierzig':40,'fünfzig':50,'sechzig':60,'siebzig':70,'achtzig':80,'neunzig':90}
    total = 0
    if 'hundert' in word:
        parts = word.split('hundert', 1)
        prefix = parts[0]; remainder = parts[1]; mult = 1
        if prefix and prefix in units: mult = units[prefix]
        total += mult * 100
        word = remainder
    if not word: return total
    if word in units: total += units[word]
    elif word in tens: total += tens[word]
    elif 'und' in word:
        p = word.split('und', 1)
        if len(p)==2: total += units.get(p[0], 0) + tens.get(p[1], 0)
    elif word.endswith('zehn'):
        prefix = word.replace('zehn','')
        if prefix in units: total += units[prefix] + 10
        elif prefix == 'sech': total += 16
        elif prefix == 'sieb': total += 17
    return total

def parse_english_number_string(text):
    units = {'zero':0,'one':1,'two':2,'three':3,'four':4,'five':5,'six':6,'seven':7,'eight':8,'nine':9,'ten':10,'eleven':11,'twelve':12,'thirteen':13,'fourteen':14,'fifteen':15}
    tens = {'twenty':20,'thirty':30,'forty':40,'fifty':50,'sixty':60,'seventy':70,'eighty':80,'ninety':90}
    multipliers = {'hundred':100, 'thousand':1000}
    words = text.split()
    has_structure = any(w in multipliers or w in tens for w in words)
    if not has_structure:
        digit_seq = []
        for w in words:
            if w in units and units[w] < 10: digit_seq.append(str(units[w]))
            elif w.isdigit(): digit_seq.append(w)
            elif w in ['point','dot']: digit_seq.append('.')
        if len(digit_seq) > 0: return "".join(digit_seq)
    current_val, total_val = 0, 0
    found_num = False
    for w in words:
        if w in units: current_val += units[w]; found_num=True
        elif w in tens: current_val += tens[w]; found_num=True
        elif w in multipliers:
            current_val = max(1, current_val) * multipliers[w]
            total_val += current_val; current_val = 0; found_num = True
    total_val += current_val
    return str(total_val) if found_num and total_val >= 0 else None

def parse_spoken_text(text, lang='de'):
    text = text.lower()
    if lang == 'de':
        text = text.replace("hundert ", "hundert").replace("tausend ", "tausend").replace(" und ", "und")
        words = text.split()
        dmap = {'eins':'1','zwei':'2','zwo':'2','drei':'3','vier':'4','fünf':'5','sechs':'6','sieben':'7','acht':'8','neun':'9','null':'0','punkt':'.'}
        digits = []
        for w in words:
            if w in dmap: digits.append(dmap[w])
            elif w.isdigit(): digits.append(w)
        if len(digits) > 1 or (len(digits)==1 and '.' in digits): return "".join(digits)
        for w in words:
            if "hundert" in w or "zig" in w or "zehn" in w or "elf" in w or "zwölf" in w:
                try: 
                    v = german_word_to_int(w)
                    if v > 0: return str(v)
                except: pass
    elif lang == 'en':
        return parse_english_number_string(text)
    return None

def load_rooms():
    rooms = {}
    if os.path.exists(CSV_PATH):
        try:
            with open(CSV_PATH, 'r') as f:
                reader = csv.reader(f); next(reader, None)
                for row in reader:
                    if len(row) > 2:
                        original_id = row[2].strip()
                        data = {
                            "id": row[0], "building": row[1], "room_id": original_id, 
                            "room_name": row[3], "floor": row[4], "wing": row[5], 
                            "category": row[6], "gender": row[7], "accessible": row[8], 
                            "notes": row[9] if len(row)>9 else ""
                        }
                        rooms[original_id] = data
                        rooms[original_id.replace('.', '')] = data
                        no_dots = original_id.replace('.', '')
                        rooms[no_dots.lstrip('0')] = data
                        if original_id == "0.10": rooms["10"] = data
        except Exception as e: rospy.logerr(f"CSV Fehler: {e}")
    else: rospy.logwarn("CSV Datei nicht gefunden!")
    return rooms

def find_special_location(category_keyword, text, rooms):
    text = text.lower()
    gender_filter = None
    if "herren" in text or "men" in text: gender_filter = "Herren"
    elif "damen" in text or "women" in text or "frauen" in text: gender_filter = "Damen"
    elif "behindert" in text or "barrierefrei" in text or "accessible" in text: gender_filter = "TRUE"
    floor_filter = None
    for fw, suffix in FLOOR_MAPPING.items():
        if fw in text: floor_filter = suffix; break
    best_match = None
    for rid, data in rooms.items():
        cat_db = data["category"]
        if category_keyword.lower() in cat_db.lower():
            if floor_filter and data["floor"] != floor_filter: continue 
            if gender_filter:
                if gender_filter == "TRUE":
                    if data["accessible"] != "TRUE": continue
                elif gender_filter not in data["gender"]: continue
            best_match = data
            break
    return best_match

def main():
    rospy.init_node('speech_control', anonymous=True)
    
    # --- PUBLISHER ÄNDERUNGEN ---
    # pub_cmd sendet jetzt SpeechCommand statt String
    pub_cmd = rospy.Publisher('/speech_command', SpeechCommand, queue_size=10)
    # pub_out sendet jetzt SpeechStatus statt String
    pub_out = rospy.Publisher('/speech_output', SpeechStatus, queue_size=10)
    
    rooms = load_rooms()
    rospy.loginfo(f"{len(rooms)} Räume geladen.")

    rec_de, rec_en = None, None
    if os.path.exists(MODEL_PATH_DE): rec_de = KaldiRecognizer(Model(MODEL_PATH_DE), 16000)
    if os.path.exists(MODEL_PATH_EN): rec_en = KaldiRecognizer(Model(MODEL_PATH_EN), 16000)

    if not rec_de and not rec_en:
        rospy.logerr("Keine Modelle!")
        sys.exit(1)

    command = ["arecord", "-D", ALSA_DEVICE, "-f", "S16_LE", "-r", "16000", "-c", "1", "-t", "raw", "-q"]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    rospy.loginfo("Speech-In BEREIT! (Monitoring gestartet)")

    last_interaction = time.time()
    timeout_sent = False

    try:
        while not rospy.is_shutdown():
            data = process.stdout.read(4000)
            if len(data) == 0: break
            
            if time.time() - last_interaction > TIMEOUT_SECONDS:
                if not timeout_sent:
                    rospy.logwarn("Timeout: 7 Sekunden Stille.")
                    
                    # --- STATUS: TIMEOUT ---
                    stat_msg = SpeechStatus()
                    stat_msg.header.stamp = rospy.Time.now()
                    stat_msg.level = "WARN"
                    stat_msg.event = "TIMEOUT"
                    stat_msg.message = "Keine Eingabe (Timeout)"
                    pub_out.publish(stat_msg)
                    
                    timeout_sent = True
                    last_interaction = time.time()
            
            res_de, res_en = "", ""
            if rec_de and rec_de.AcceptWaveform(data):
                res_de = json.loads(rec_de.Result()).get('text', '')
            if rec_en and rec_en.AcceptWaveform(data):
                res_en = json.loads(rec_en.Result()).get('text', '')

            if res_de: print(f"[DE]: {res_de}")
            if res_en: print(f"[EN]: {res_en}")

            winner_text, winner_lang = None, None
            if any(t in res_de for t in TRIGGERS_DE): winner_text, winner_lang = res_de, 'de'
            elif any(t in res_en for t in TRIGGERS_EN): winner_text, winner_lang = res_en, 'en'
            
            if not winner_text:
                pid = parse_spoken_text(res_de, 'de')
                if pid and pid in rooms: winner_text, winner_lang = res_de, 'de'

            if winner_text:
                last_interaction = time.time()
                timeout_sent = False
                found_room_data = None
                
                for kw, cat in CATEGORY_MAPPING.items():
                    if kw in winner_text:
                        found_room_data = find_special_location(cat, winner_text, rooms); break
                
                if not found_room_data:
                    parsed_id = parse_spoken_text(winner_text, winner_lang)
                    if parsed_id:
                        if parsed_id in rooms: found_room_data = rooms[parsed_id]
                        elif parsed_id.replace('.','') in rooms: found_room_data = rooms[parsed_id.replace('.','')]

                if found_room_data:
                    # --- BEFEHL SENDEN (COMMAND) ---
                    cmd_msg = SpeechCommand()
                    cmd_msg.header.stamp = rospy.Time.now()
                    cmd_msg.target = "room"
                    cmd_msg.room_id = found_room_data["room_id"]
                    cmd_msg.floor = found_room_data["floor"]
                    cmd_msg.wing = found_room_data["wing"]
                    
                    rospy.loginfo(f"--> TREFFER: Raum {cmd_msg.room_id}")
                    pub_cmd.publish(cmd_msg)
                    
                else:
                    rospy.logwarn(f"Verstanden: '{winner_text}', aber Ziel unbekannt.")
                    
                    # --- STATUS SENDEN: UNKNOWN ---
                    stat_msg = SpeechStatus()
                    stat_msg.header.stamp = rospy.Time.now()
                    stat_msg.level = "WARN"
                    stat_msg.event = "UNKNOWN_ROOM"
                    stat_msg.transcript = winner_text
                    stat_msg.message = "Raum nicht gefunden"
                    pub_out.publish(stat_msg)

    except Exception as e:
        rospy.logerr(f"Fehler: {e}")
    finally:
        process.terminate()

if __name__ == '__main__':
    main()
