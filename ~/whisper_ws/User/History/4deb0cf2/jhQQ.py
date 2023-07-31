#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from audio_common_msgs.msg import AudioData
import sounddevice as sd
import numpy as np

def audio_callback(indata, frames, time, status):
    audio_data = (indata * 32767).astype(np.int16).tobytes()
    audio_msg = AudioData(data=audio_data)
    pub.publish(audio_msg)

if __name__ == '__main__':
    rospy.init_node('microphone_audio_publisher', anonymous=True)
    pub = rospy.Publisher('audio_topic', AudioData, queue_size=10)

    samplerate = 44100  # Adjust this to match the sample rate of your microphone
    channels = 1  # Adjust this to match the number of channels of your microphone

    with sd.InputStream(callback=audio_callback, channels=channels, samplerate=samplerate):
        rospy.spin()
