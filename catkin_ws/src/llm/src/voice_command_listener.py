import rospy
import openai
import speech_recognition as sr
from std_msgs.msg import String
from dotenv import load_dotenv
import os
import re

# Load environment variables from .env file
load_dotenv()

# Initialize OpenAI with the API key from the .env file
openai.api_key = os.getenv('OPENAI_API_KEY')

# List of colors to check for in the speech input
color_list = [
    "red", "green", "blue", "yellow", "pink", "orange", "purple", "brown", 
    "black", "white", "gray", "violet", "cyan", "magenta", "beige"
]

def voice_to_text():
    # Initialize recognizer and microphone
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    print("Say something...")

    with mic as source:
        # Adjust for ambient noise
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)
    
    try:
        # Use Google Speech Recognition to convert audio to text
        print("Recognizing...")
        command = recognizer.recognize_google(audio)
        print("You said:", command)
        return command
    except sr.UnknownValueError:
        print("Sorry, I could not understand the audio")
        return ""
    except sr.RequestError:
        print("Could not request results from Google Speech Recognition service")
        return ""

def extract_color(text):
    """
    Extract color from the given text. Checks for known color names.
    Returns the color name if found, or None if no color is detected.
    """
    # Use regular expressions to find a color in the text
    for color in color_list:
        if re.search(r'\b' + color + r'\b', text, re.IGNORECASE):
            return color
    return None

def send_to_openai(text):
    try:
        response = openai.Completion.create(
            engine="text-davinci-003",  # You can choose a different model if needed
            prompt=text,
            max_tokens=100
        )
        message = response.choices[0].text.strip()
        return message
    except openai.error.OpenAIError as e:
        print(f"OpenAI API Error: {e}")
        return ""

def talker():
    # Initialize ROS node
    rospy.init_node('voice_command_color_node', anonymous=True)
    pub = rospy.Publisher('color', String, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Get voice command from the microphone
        command = voice_to_text()

        if command:
            # Send the voice command to OpenAI for processing
            openai_response = send_to_openai(command)

            # Try to extract the color from the response or command
            color = extract_color(openai_response)
            if color:
                rospy.loginfo(f"Detected color: {color}")
                pub.publish(color)
            else:
                rospy.loginfo("No color detected in the command.")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
