from pydub import AudioSegment
import os

directory = "."
for filename in os.listdir(directory):
    if filename.endswith(".wav"):
        info = AudioSegment.from_wav(filename)
        print(info.frame_rate)
        if(info.frame_rate == 48000):
            os.remove(filename)
