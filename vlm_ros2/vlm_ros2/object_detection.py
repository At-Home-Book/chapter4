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

class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')

        self.client = OpenAI(
            # This is the default and can be omitted
            api_key=os.environ.get("OPENAI_API_KEY"),
            base_url="https://api.openai.com/v1",
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
        prompt = "Use boxes to locate each object in the image and describe its characteristics. Output the coordinates of all bboxes in JSON format. Do not output the ```json``` code segment."
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

    object_detection = ObjectDetection()

    rclpy.spin(object_detection)

    object_detection.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
