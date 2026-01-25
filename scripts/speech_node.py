#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import subprocess
import rospy
import time
import csv
from std_msgs.msg import String
from vosk import Model, KaldiRecognizer

# --- ROS PARAMETER LADEN (Statt Hardcoding) ---
# Wir holen die Pfade jetzt dynamisch.
# Wenn nichts kommt, nutzen wir relative Pfade als Fallback.

def get_params():
    # Helper um Pfade relativ zum Paket zu finden, falls nötig
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Defaults (relativ)
    default_model_de = os.path.join(base_dir, "models/model_de")
    default_model_en = os.path.join(base_dir, "models/model_en")
    default_csv = os.path.join(base_dir, "config/rooms.csv")
    
    # ROS Parameter lesen (Private Namespace '~')
    model_de = rospy.get_param('~model_path_de', default_model_de)
    model_en = rospy.get_param('~model_path_en', default_model_en)
    csv_path = rospy.get_param('~csv_path', default_csv)
    
    # Device ID: Kann "1" (int) oder "plughw:1,0" (string) sein.
    # Dein alter Code nutzt "arecord", das braucht Strings wie "plughw:1,0".
    # Wenn wir nur "1" bekommen, basteln wir "plughw:1,0" daraus.
    device_arg = rospy.get_param('~device_index', "plughw:1,0")
    if str(device_arg).isdigit():
        alsa_device = f"plughw:{device_arg},0"
    else:
        alsa_device = str(device_arg)
        
    return model_de, model_en, csv_path, alsa_device

# Parameter holen (global für Zugriff in Funktionen, wenn nötig, aber besser in main)
# Wir setzen diese Variablen später in der main() richtig.

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

# --- PARSER (Dein Original-Code) ---
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

def load_rooms(csv_path):
    rooms = {}
    if os.path.exists(csv_path):
        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f, delimiter=';'); # Sicherheitshalber Semikolon oder Auto-Detect
                # Header skippen ist tricky, wenn CSV keinen hat. Dein Code machte next(reader, None).
                # Besser: Wir schauen uns die erste Zeile an.
                try:
                    header = next(reader, None)
                    # Falls erste Zeile schon Daten sind (ID ist Zahl), dann nutzen.
                    # Aber meistens ist Header da.
                except StopIteration:
                    return {}

                for row in reader:
                    # Dein CSV Parser ist sehr spezifisch auf Spalten-Index.
                    # Wir lassen das so, hoffen dass CSV Format stabil ist.
                    if len(row) > 2:
                        # Index-Check gegen IndexOutOfRange
                        rid = row[0] if len(row) > 0 else ""
                        bid = row[1] if len(row) > 1 else ""
                        oid = row[2].strip() if len(row) > 2 else ""
                        rname = row[3] if len(row) > 3 else ""
                        fl = row[4] if len(row) > 4 else ""
                        wg = row[5] if len(row) > 5 else ""
                        cat = row[6] if len(row) > 6 else ""
                        gen = row[7] if len(row) > 7 else ""
                        acc = row[8] if len(row) > 8 else ""
                        notiz = row[9] if len(row) > 9 else ""

                        data = {
                            "id": rid, "building": bid, "room_id": oid, 
                            "room_name": rname, "floor": fl, "wing": wg, 
                            "category": cat, "gender": gen, "accessible": acc, 
                            "notes": notiz
                        }
                        rooms[oid] = data
                        rooms[oid.replace('.', '')] = data
                        no_dots = oid.replace('.', '')
                        rooms[no_dots.lstrip('0')] = data
                        if oid == "0.10": rooms["10"] = data
        except Exception as e: rospy.logerr(f"CSV Fehler: {e}")
    else: rospy.logwarn(f"CSV Datei nicht gefunden: {csv_path}")
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
    
    # PARAMETER LADEN (Jetzt dynamisch!)
    model_path_de, model_path_en, csv_path_val, alsa_device_val = get_params()

    rospy.loginfo(f"Konfiguration: Device={alsa_device_val}, CSV={csv_path_val}")

    pub_cmd = rospy.Publisher('/speech_command', String, queue_size=10)
    pub_out = rospy.Publisher('/speech_output', String, queue_size=10)
    
    rooms = load_rooms(csv_path_val)
    rospy.loginfo(f"{len(rooms)} Räume geladen.")

    rec_de, rec_en = None, None
    
    # Modelle laden mit besserem Error Handling
    if os.path.exists(model_path_de): 
        try:
            rec_de = KaldiRecognizer(Model(model_path_de), 16000)
        except Exception as e:
            rospy.logerr(f"Konnte DE Modell nicht laden: {e}")
            
    if os.path.exists(model_path_en): 
        try:
            rec_en = KaldiRecognizer(Model(model_path_en), 16000)
        except Exception as e:
            rospy.logerr(f"Konnte EN Modell nicht laden: {e}")

    if not rec_de and not rec_en:
        rospy.logerr("KEINE MODELLE GELADEN! Pfade prüfen.")
        sys.exit(1)

    # ARECORD STARTEN
    # Wichtig: alsa_device_val kommt jetzt aus der Launch File
    command = ["arecord", "-D", alsa_device_val, "-f", "S16_LE", "-r", "16000", "-c", "1", "-t", "raw", "-q"]
    
    try:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except Exception as e:
        rospy.logerr(f"Konnte 'arecord' nicht starten: {e}")
        sys.exit(1)

    rospy.loginfo("Speech-In BEREIT! (Monitoring gestartet)")

    last_interaction = time.time()
    timeout_sent = False

    try:
        while not rospy.is_shutdown():
            data = process.stdout.read(4000)
            if len(data) == 0: 
                # Prozess evtl gestorben?
                if process.poll() is not None:
                    rospy.logerr("Audio-Prozess beendet.")
                    break
                continue
            
            # Timeout Logik
            if time.time() - last_interaction > TIMEOUT_SECONDS:
                if not timeout_sent:
                    # Nur Warnung, kein Abbruch
                    # rospy.logwarn("Timeout: 7 Sekunden Stille.")
                    msg = json.dumps({"event": "timeout"}, ensure_ascii=False)
                    pub_out.publish(String(msg))
                    timeout_sent = True
                    last_interaction = time.time()
            
            res_de, res_en = "", ""
            if rec_de and rec_de.AcceptWaveform(data):
                res_de = json.loads(rec_de.Result()).get('text', '')
            if rec_en and rec_en.AcceptWaveform(data):
                res_en = json.loads(rec_en.Result()).get('text', '')

            if res_de: 
                rospy.loginfo(f"[DE]: {res_de}")
            if res_en: 
                rospy.loginfo(f"[EN]: {res_en}")

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
                    final_cmd = {
                        "room_id": found_room_data["room_id"],
                        "floor": found_room_data["floor"],
                        "wing": found_room_data["wing"]
                    }
                    json_str = json.dumps(final_cmd, ensure_ascii=False)
                    rospy.loginfo(f"--> TREFFER: {json_str}")
                    pub_cmd.publish(String(json_str))
                else:
                    rospy.logwarn(f"Verstanden: '{winner_text}', aber Ziel unbekannt.")
                    error_msg = {
                        "event": "unknown_room",
                        "transcript": winner_text
                    }
                    pub_out.publish(String(json.dumps(error_msg, ensure_ascii=False)))

    except Exception as e:
        rospy.logerr(f"Fehler in Main Loop: {e}")
    finally:
        process.terminate()

if __name__ == '__main__':
    main()
