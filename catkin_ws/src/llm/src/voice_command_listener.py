#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import speech_recognition as sr

def process_command(command):
    # Placeholder for LLM processing; modify this part as needed
    # Here we are simply returning the command after basic processing.
    processed_command = command.lower()  # Example transformation (e.g., lowercase)
    rospy.loginfo(f"Processed Command: {processed_command}")
    return processed_command

def listen_and_publish():
    rospy.init_node('voice_command_listener', anonymous=True)
    pub = rospy.Publisher('command', String, queue_size=10)

    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    rospy.loginfo("Listening for voice commands...")

    while not rospy.is_shutdown():
        with mic as source:
            recognizer.adjust_for_ambient_noise(source)
            rospy.loginfo("Say something!")
            audio = recognizer.listen(source)

        try:
            # Transcribe audio to text
            command = recognizer.recognize_google(audio)
            rospy.loginfo(f"Heard: {command}")

            # Process the transcribed text with LLM or any language processing logic
            processed_command = process_command(command)

            # Publish to the 'command' topic
            pub.publish(processed_command)

        except sr.UnknownValueError:
            rospy.logwarn("Could not understand audio")
        except sr.RequestError as e:
            rospy.logerr(f"Could not request results from the speech recognition service; {e}")

if __name__ == '__main__':
    try:
        listen_and_publish()
    except rospy.ROSInterruptException:
        pass
