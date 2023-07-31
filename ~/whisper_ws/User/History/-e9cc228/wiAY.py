import sounddevice as sd

def print_audio_devices():
    devices = sd.query_devices()
    print("Available audio input devices:")
    for i, device in enumerate(devices):
        if device['max_input_channels'] > 0:
            print(f"{i}. {device['name']}, Sample Rate: {device['default_samplerate']} Hz")

if __name__ == '__main__':
    print_audio_devices()