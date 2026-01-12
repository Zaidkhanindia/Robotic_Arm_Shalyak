from speech_to_text import SpeechToText
import csv
import time

CSV_PATH = "/home/zaid-khan/command.csv"

class TextToCommand:
    def __init__(self):
        self.speech = SpeechToText()

    def get_joints_angle_from_text(self):
        text = self.speech.get_text_from_speech()
        if not text:
            return None, None

        text = text.lower().strip()
        tokens = text.split()

        if text == 'ready':
            print('Yes Ready....')
            return None, None

        joint = None
        angle = None

        for token in tokens:
            if token in ['alpha', 'beta', 'gamma']:
                joint = token
            elif token.isdigit():
                angle = int(token)

        return joint, angle

    def write_command(self, joint, angle):
        if joint is None or angle is None:
            return

        with open(CSV_PATH, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([joint, angle, int(time.time())])

        print(f"Written to CSV: {joint}, {angle}")


cmd = TextToCommand()

while True:
    joint, angle = cmd.get_joints_angle_from_text()
    print("Detected:", joint, angle)
    cmd.write_command(joint, angle)
    time.sleep(0.5)
