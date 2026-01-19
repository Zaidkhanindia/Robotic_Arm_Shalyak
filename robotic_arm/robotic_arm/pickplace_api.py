from speech_to_text import SpeechToText
import csv
import time

CSV_PATH = "/home/zaid-khan/pickplace.csv"

class TextToCommand:
    def __init__(self):
        self.speech = SpeechToText()

    def get_data_from_text(self):
        text = self.speech.get_text_from_speech()

        if not text:
            return None, None

        text = text.lower().strip()
        tokens = text.split()

        command = None
        gripper = None

        for token in tokens:
            if token in ['pick', 'drag']:
                command = 'pick'
            elif token in ['place', 'plays', 'please', 'drop']:
                command = 'place'
            elif token in ['open', 'close']:
                gripper = token

        return command, gripper

    def write_command(self, command, gripper):

        with open(CSV_PATH, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([command, gripper])

        print(f"Written to CSV: {command}, {gripper}")


cmd = TextToCommand()

while True:
    command, gripper = cmd.get_data_from_text()

    if command is None and gripper is None:
        print("Invalid / unclear command, ignoring...")
        time.sleep(1.0)
        continue

    print("Detected: ", command, gripper)
    cmd.write_command(command, gripper)
    time.sleep(2.0)

#export GOOGLE_APPLICATION_CREDENTIALS="/home/zaid-khan/Downloads/voice-controlled-robot-483812-dc7d071fc002.json"

