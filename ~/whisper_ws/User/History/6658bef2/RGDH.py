#!/usr/bin/env python3
import rospy
import numpy as np
import subprocess

from sensor_msgs.msg import String
from audio_common_msgs.msg import AudioData
sys.path.append("/whisper_ws/src/whisper/")
from ../.. import whisper

class whisper_ros():
    def __init__(self): 
        rospy.init_node("whisper_ros", anonymous=True)
        self.image_sub = rospy.Subscriber("/Audiodata", AudioData, self.audio_callback)
        self.whisper_txt_pub = rospy.Publisher("/whisper_txt", String, queue_size=10)


    def audio_callback(self, audio_msg):

        audio_file_path = '/tmp/received_audio.mp3'

        with open(audio_file_path, 'wb') as f:
            f.write(audio_msg.data)

        # load model
        model = whisper.load_model("base")

        # load audio and pad/trim it to fit 30 seconds
        audio = whisper.load_audio("/tmp/received_audio.mp3")
        audio = whisper.pad_or_trim(audio)

        # make log-Mel spectrogram and move to the same device as the model
        mel = whisper.log_mel_spectrogram(audio).to(model.device)

        # detect the spoken language
        _, probs = model.detect_language(mel)
        print(f"Detected language: {max(probs, key=probs.get)}")

        # decode the audio
        options = whisper.DecodingOptions()
        result = whisper.decode(model, mel, options)

        # Publish the recognized text
        text_msg = String()
        text_msg.data = result.txt
        self.text_pub.publish(text_msg)

        # Remove temporary files
        os.remove(audio_file_path)

        # print the recognized text
        print(result.text)


if __name__ == "__main__":
    try:
        wr = whisper_ros()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass