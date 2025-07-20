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

class SignageRecognition(Node):

    def __init__(self):
        super().__init__('signage_recognition')

        self.client = OpenAI(
            # Set API Key and Base URL
            api_key=os.getenv('DASHSCOPE_API_KEY'),
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
        )
        
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, 
            "/image_raw", 
            self.image_callback , 
            qos_profile_sensor_data)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            global photo_taken
            if photo_taken == False:
                cv2.imwrite("photo.png", cv_image)
                photo_taken = True
                self.get_logger().info("The photo is taken.")
                # Get response from VLM
                self.get_logger().info("Getting response from VLM...")
                self.get_response()
        except Exception as e:
            self.get_logger().error('cv_bridge exception: %s' % e)

    def get_response(self):
        # Input text
        prompt = "What is the signage in this image?"
        with open("photo.png", "rb") as image_file:
            base64_image = base64.b64encode(image_file.read()).decode("utf-8")

        completion = self.client.chat.completions.create(
            # Set Model
            model="qwen-vl-max-latest",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {"type": "image_url", "image_url": {"url": f"data:image/png;base64,{base64_image}"},}
                    ],
                }
            ],
        )

        self.get_logger().info(completion.choices[0].message.content)
        cv_photo = cv2.imread("photo.png")
        cv2.imshow('Photo',cv_photo)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)

    signage_recognition = SignageRecognition()

    rclpy.spin(signage_recognition)

    signage_recognition.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
