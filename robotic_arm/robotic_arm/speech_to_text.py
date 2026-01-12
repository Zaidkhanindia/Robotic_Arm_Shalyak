from google.cloud import speech
import pyaudio
import queue

RATE = 16000
CHUNK = int(RATE / 10)

class SpeechToText:
    def __init__(self):
        self.audio_queue = queue.Queue()
        self.client = speech.SpeechClient()

        self.config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=RATE,
            language_code="en-US",
        )

        self.streaming_config = speech.StreamingRecognitionConfig(
            config=self.config,
            interim_results=False,
        )

        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK,
            stream_callback=self.callback,
        )

    def callback(self, in_data, frame_count, time_info, status):
        self.audio_queue.put(in_data)
        return None, pyaudio.paContinue

    def request_generator(self):
        while True:
            data = self.audio_queue.get()
            yield speech.StreamingRecognizeRequest(audio_content=data)

    def get_text_from_speech(self):
        responses = self.client.streaming_recognize(
            self.streaming_config,
            self.request_generator()
        )

        for response in responses:
            for result in response.results:
                return result.alternatives[0].transcript
