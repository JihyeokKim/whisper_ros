#!/usr/bin/env python3
import rospy
import numpy as np
import argparse
import io
import os
import speech_recognition as sr
import whisper
import torch, gc

from datetime import datetime, timedelta
from queue import Queue
from tempfile import NamedTemporaryFile
from time import sleep
from sys import platform

from std_msgs.msg import String

class whisper_ros_realtime():
    def __init__(self): 
        rospy.init_node("whisper_ros_realtime", anonymous=True)
        self.text_pub = rospy.Publisher("/whisper_txt", String, queue_size=10)
        
        
        parser = argparse.ArgumentParser()
        parser.add_argument("--model", default="medium", help="Model to use",
                            choices=["tiny", "base", "small", "medium", "large"])
        parser.add_argument("--non_english", action='store_true',
                            help="Don't use the english model.")
        parser.add_argument("--energy_threshold", default=1000,
                            help="Energy level for mic to detect.", type=int)
        parser.add_argument("--record_timeout", default=3,
                            help="How real time the recording is in seconds.", type=float)
        parser.add_argument("--phrase_timeout", default=5,
                            help="How much empty space between recordings before we "
                                "consider it a new line in the transcription.", type=float)  
        if 'linux' in platform:
            parser.add_argument("--default_microphone", default='pulse',
                                help="Default microphone name for SpeechRecognition. "
                                    "Run this with 'list' to view available Microphones.", type=str)
        args = parser.parse_args()
        
        # The last time a recording was retreived from the queue.
        phrase_time = None
        # Current raw audio bytes.
        last_sample = bytes()
        # Thread safe Queue for passing data from the threaded recording callback.
        data_queue = Queue()
        # We use SpeechRecognizer to record our audio because it has a nice feauture where it can detect when speech ends.
        recorder = sr.Recognizer()
        recorder.energy_threshold = args.energy_threshold
        # Definitely do this, dynamic energy compensation lowers the energy threshold dramtically to a point where the SpeechRecognizer never stops recording.
        recorder.dynamic_energy_threshold = False
        
        # Important for linux users. 
        # Prevents permanent application hang and crash by using the wrong Microphone
        if 'linux' in platform:
            print("linux")
            mic_name = args.default_microphone
            if not mic_name or mic_name == 'list':
                print("Available microphone devices are: ")
                for index, name in enumerate(sr.Microphone.list_microphone_names()):
                    print(f"Microphone with name \"{name}\" found")   
                return
            else:
                for index, name in enumerate(sr.Microphone.list_microphone_names()):
                    if mic_name in name:
                        print("Mic: ", mic_name)
                        source = sr.Microphone(sample_rate=48000, device_index=index)
                        break
        else:
            source = sr.Microphone(sample_rate=48000)
            
        # Load / Download model
        model = args.model
        if args.model != "large" and not args.non_english:
            model = model + ".en"
        audio_model = whisper.load_model(model)

        record_timeout = args.record_timeout
        phrase_timeout = args.phrase_timeout

        temp_file = NamedTemporaryFile().name
        transcription = ['']

        with source:
            recorder.adjust_for_ambient_noise(source)

        def record_callback(_, audio:sr.AudioData) -> None:
            """
            Threaded callback function to recieve audio data when recordings finish.
            audio: An AudioData containing the recorded bytes.
            """
            # Grab the raw bytes and push it into the thread safe queue.
            data = audio.get_raw_data()
            data_queue.put(data)

    
        # Create a background thread that will pass us raw audio bytes.
        # We could do this manually but SpeechRecognizer provides a nice helper.
        recorder.listen_in_background(source, record_callback, phrase_time_limit=record_timeout)

        # Cue the user that we're ready to go.
        print("Model loaded.\n")

        while True:
            try:
                now = datetime.utcnow()
                # Pull raw recorded audio from the queue.
                if not data_queue.empty():
                    phrase_complete = False
                    # If enough time has passed between recordings, consider the phrase complete.
                    # Clear the current working audio buffer to start over with the new data.
                    if phrase_time and now - phrase_time > timedelta(seconds=phrase_timeout):
                        last_sample = bytes()
                        phrase_complete = True
                    # This is the last time we received new audio data from the queue.
                    phrase_time = now

                    # Concatenate our current audio data with the latest audio data.
                    while not data_queue.empty():
                        data = data_queue.get()
                        last_sample += data

                    # Use AudioData to convert the raw data to wav data.
                    audio_data = sr.AudioData(last_sample, source.SAMPLE_RATE, source.SAMPLE_WIDTH)
                    wav_data = io.BytesIO(audio_data.get_wav_data())

                    # Write wav data to the temporary file as bytes.
                    with open(temp_file, 'w+b') as f:
                        f.write(wav_data.read())

                    # Read the transcription.
                    result = audio_model.transcribe(temp_file, fp16=torch.cuda.is_available())
                    text = result['text'].strip()

                    # Publish the recognized text
                    text_msg = String()
                    text_msg.data = text
                    self.text_pub.publish(text_msg)

                    # If we detected a pause between recordings, add a new item to our transcripion.
                    # Otherwise edit the existing one.
                    if phrase_complete:
                        transcription.append(text)
                    else:
                        transcription[-1] = text

                    # Clear the console to reprint the updated transcription.
                    os.system('cls' if os.name=='nt' else 'clear')
                    for line in transcription:
                        print(line)
                    # Flush stdout.
                    print('', end='', flush=True)

                    # Check shutdown
                    if rospy.is_shutdown():
                        break

                    # Infinite loops are bad for processors, must sleep.
                    sleep(0.25)
                    
            except KeyboardInterrupt:
                break

        print("\n\nTranscription:")
        for line in transcription:
            print(line)
        


if __name__ == "__main__":
    wrr = whisper_ros_realtime()
    rospy.spin()