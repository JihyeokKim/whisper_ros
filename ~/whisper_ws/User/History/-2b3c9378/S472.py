#!/usr/bin/env python3

import rospy
from audio_common_msgs.msg import AudioData

def publish_mp3_as_audiodata(mp3_file_path, topic_name='audio_topic'):
    rospy.init_node('mp3_publisher_node', anonymous=True)
    pub = rospy.Publisher(topic_name, AudioData, queue_size=10)
    rate = rospy.Rate(10)  # Modify the publishing rate as needed

    with open(mp3_file_path, 'rb') as mp3_file:
        mp3_data = mp3_file.read()

    while not rospy.is_shutdown():
        audio_msg = AudioData(data=mp3_data)
        pub.publish(audio_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        mp3_file_path = '/whisper_ws/src/whisper/whisper/apron_sample.mp3'  # Replace with the path to your MP3 file
        publish_mp3_as_audiodata(mp3_file_path)
    except rospy.ROSInterruptException:
        pass