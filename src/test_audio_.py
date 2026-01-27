#!/usr/bin/env python3
import rospy
from robot_toolkit_msgs.srv import say_to_file_srv, say_to_file_srvResponse
from pydub import AudioSegment
import simpleaudio as sa
import numpy as np
from task_module import Task_module as tm

def call_say_to_file_srv(text):
    rospy.wait_for_service('pytoolkit/ALTextToSpeech/say_to_file_srv')
    
    try:
        say_to_file_service = rospy.ServiceProxy('pytoolkit/ALTextToSpeech/say_to_file_srv', say_to_file_srv)
        response = say_to_file_service(text)
        
        if response.data:
            print(f"Received audio data of length: {len(response.data)}")

            # Convert the response data to a byte array
            audio_bytes = bytearray(response.data)
            
            # Create an AudioSegment from raw audio data
            audio_segment = AudioSegment(
                audio_bytes,
                sample_width=2,  # Assuming 8-bit audio (change if different)
                frame_rate=22050,  # Set this to the actual sample rate
                channels=1  # Set to 1 for mono, 2 for stereo
            )

            # Play the audio
            play_obj = sa.play_buffer(audio_segment.raw_data, num_channels=1, 
                                      bytes_per_sample=audio_segment.sample_width, 
                                      sample_rate=audio_segment.frame_rate)
            play_obj.wait_done()  # Wait until audio is finished playing
            
            # Save the audio data as a WAV file
            audio_segment.export('/tmp/received_audio.wav', format='wav')
            print("Audio data saved to /tmp/received_audio.wav")
        else:
            print("No audio data received.")
    
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    
    text_to_say = "Hola, soy nova"
    
    tm = tm(perception = False)
    call_say_to_file_srv(text_to_say)
