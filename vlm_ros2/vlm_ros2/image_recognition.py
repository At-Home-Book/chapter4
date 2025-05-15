import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
import base64
from openai import OpenAI

photo_taken = False

class ImageRecognition(Node):

    def __init__(self):
        super().__init__('image_recognition')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, 
            "/image_raw", 
            self.image_callback , 
            qos_profile_sensor_data)
    
        self.client = OpenAI(
            # This is the default and can be omitted
            api_key=os.environ.get("OPENAI_API_KEY"),
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow('Camera', cv_image)
            cv2.waitKey(1)
            global photo_taken
            if photo_taken == False:
                cv2.imwrite("photo.png", cv_image)
                photo_taken = True
                self.get_logger().info("The photo is taken.")
                self.get_response()
        except Exception as e:
            self.get_logger().error('cv_bridge exception: %s' % e)

    def get_response(self):
        prompt = "What is in this image?"
        with open("photo.png", "rb") as image_file:
            b64_image = base64.b64encode(image_file.read()).decode("utf-8")

        response = self.client.responses.create(
            model="gpt-4o-mini",
            input=[
                {
                    "role": "user",
                    "content": [
                        {"type": "input_text", "text": prompt},
                        {"type": "input_image", "image_url": f"data:image/png;base64,{b64_image}"},
                    ],
                }
            ],
        )

        self.get_logger().info(response.output_text)

def main(args=None):
    rclpy.init(args=args)

    image_recognition = ImageRecognition()

    rclpy.spin(image_recognition)

    image_recognition.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
