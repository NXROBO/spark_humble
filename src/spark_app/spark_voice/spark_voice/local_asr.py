#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import whisper
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write

Model = 'base'     # Whisper model size (tiny, base, small, medium, large)
Translate = False   # Translate non-English to English?
SampleRate = 44100  # Stream device recording frequency
BlockSize = 30      # Block size in milliseconds
Threshold = 0.1     # Minimum volume threshold to activate listening
Vocals = [50, 1000] # Frequency range to detect sounds that could be speech
EndBlocks = 40      # Number of blocks to wait before sending to Whisper

class SpeechRecognition(Node):
    def __init__(self):
        super().__init__('SpeechRecognition')
        self.declare_parameter('language', 'zh')
        self.lang=self.get_parameter('language').get_parameter_value().string_value 
        self.running = True
        self.padding = 0
        self.prevblock = self.buffer = np.zeros((0,1))
        self.fileready = False
        self.get_logger().info("Loading Whisper Model..")
        self.model = whisper.load_model(Model)
        self.publisher_ = self.create_publisher(String, '/voice/stt', 10)
        

    def callback(self, indata, frames, time, status):
        #if status: print(status) # for debugging, prints stream errors.
        if not any(indata):
            return
        # A few alternative methods exist for detecting speech.. #indata.max() > Threshold
        #zero_crossing_rate = np.sum(np.abs(np.diff(np.sign(indata)))) / (2 * indata.shape[0]) # threshold 20
        freq = np.argmax(np.abs(np.fft.rfft(indata[:, 0]))) * SampleRate / frames
        if np.sqrt(np.mean(indata**2)) > Threshold and Vocals[0] <= freq <= Vocals[1]:
            print('.', end='', flush=True)
            if self.padding < 1: self.buffer = self.prevblock.copy()
            self.buffer = np.concatenate((self.buffer, indata))
            self.padding = EndBlocks
        else:
            self.padding -= 1
            if self.padding > 1:
                self.buffer = np.concatenate((self.buffer, indata))
            elif self.padding < 1 < self.buffer.shape[0] > SampleRate: # if enough silence has passed, write to file.
                self.fileready = True
                write('dictate.wav', SampleRate, self.buffer) # I'd rather send data to Whisper directly..
                self.buffer = np.zeros((0,1))
            elif self.padding < 1 < self.buffer.shape[0] < SampleRate: # if recording not long enough, reset buffer.
                self.buffer = np.zeros((0,1))
            else:
                self.prevblock = indata.copy() #np.concatenate((self.prevblock[-int(SampleRate/10):], indata)) # SLOW

    def process(self):
        if self.fileready:
            self.get_logger().info("Transcribing {}..".format(self.lang))
            result = self.model.transcribe('dictate.wav',fp16=False,language=self.lang,task='translate' if Translate else 'transcribe')
            self.get_logger().info(result['text'])
            msg = String()
            msg.data = result['text']
            self.publisher_.publish(msg)
            self.fileready = False

    def listen(self):
        self.get_logger().info("Listening..")
        with sd.InputStream(channels=1, callback=self.callback, blocksize=int(SampleRate * BlockSize / 1000), samplerate=SampleRate):
            while self.running : self.process()


def main(args=None):
    rclpy.init(args=args)

    sr = SpeechRecognition()
    sr.listen()

    rclpy.spin(sr)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sr.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

