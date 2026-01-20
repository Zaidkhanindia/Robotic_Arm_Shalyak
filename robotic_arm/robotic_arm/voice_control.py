from speech_to_text import SpeechToText
import csv
import time

CSV_PATH = "/home/zaid-khan/command.csv"

class TextToCommand:
    def __init__(self):
        self.speech = SpeechToText()

    def get_data_from_text(self):
        text = self.speech.get_text_from_speech()

        if not text:
            return None, None, None

        text = text.lower().strip()
        tokens = text.split()

        joint = None
        angle = None
        command = None
        sign = None

        for token in tokens:
            if token in ['alpha', 'beta', 'gamma', 'delta']:
                joint = token
            elif token in ['open', 'close']:
                command = token
            elif token in ['minus', '-']:
                sign = '-'
            elif token.isdigit():
                if sign == '-':
                    angle = int('-' + token)
                else:
                    angle = int(token)

        return joint, angle, command

    def write_command(self, joint, angle, command):

        with open(CSV_PATH, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([joint, angle, command])

        print(f"Written to CSV: {joint}, {angle}, {command}")


cmd = TextToCommand()

while True:
    joint, angle, command = cmd.get_data_from_text()

    if (joint is None or angle is None) and command is None:
        print("Invalid / unclear command, ignoring...")
        time.sleep(0.3)
        continue

    cmd.write_command(joint, angle, command)
    time.sleep(0.4)

