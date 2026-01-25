🎤 Speech-In: Setup Guide
Dieser Guide erklärt, wie man das Speech-Modul auf einem frischen System (z.B. neu geflashter Raspberry Pi) komplett neu installiert.

1. System vorbereiten
Installiere Audio-Treiber und Python-Bibliotheken:

bash
sudo apt update
sudo apt install -y alsa-utils python3-pip git
pip3 install vosk sounddevice numpy
2. Code herunterladen
Klone das Repository in deinen Workspace:

bash
cd ~/catkin_ws/src
git clone https://github.com/hwr-sew25/Speech-In.git speech_in
3. Sprachmodelle installieren (WICHTIG!)
Die Modelle sind nicht im Git. Du musst sie manuell laden:

bash
# Ordner erstellen
mkdir -p ~/catkin_ws/src/speech_in/models
cd ~/catkin_ws/src/speech_in/models

# Deutsches Modell laden & entpacken
wget https://alphacephei.com/vosk/models/vosk-model-small-de-0.15.zip
unzip vosk-model-small-de-0.15.zip
mv vosk-model-small-de-0.15 model_de
rm vosk-model-small-de-0.15.zip

# Englisches Modell laden & entpacken
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
mv vosk-model-small-en-us-0.15 model_en
rm vosk-model-small-en-us-0.15.zip
4. Bauen & Starten
Jetzt alles kompilieren und starten:

bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Starten (Mikrofon Index ggf. anpassen, z.B. mic:=1)
roslaunch speech_in speech.launch mic:=1
