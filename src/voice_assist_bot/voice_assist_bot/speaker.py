import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker_node')
        self.publisher = self.create_publisher(String, 'voice_commands', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.get_logger().info("Speaker Node Initialized.")

    def listen_and_publish(self):
        with self.microphone as source:
            self.get_logger().info("Listening for commands...")
            self.recognizer.adjust_for_ambient_noise(source)
            audio = self.recognizer.listen(source)

        try:
            command = self.recognizer.recognize_google(audio).lower()
            self.get_logger().info(f"Recognized command: {command}")
            msg = String()
            msg.data = command
            self.publisher.publish(msg)
        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand the command.")
        except sr.RequestError:
            self.get_logger().warn("Could not request results, check internet connection.")

def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    while rclpy.ok():
        node.listen_and_publish()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()