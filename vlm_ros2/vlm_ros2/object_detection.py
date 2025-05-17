import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os
import base64
from openai import OpenAI
import json

photo_taken = False

class ObjectDetection(Node):

    def __init__(self):
        super().__init__('object_detection')

        self.client = OpenAI(
            #api_key=os.environ.get("OPENAI_API_KEY"),
            #base_url="https://api.openai.com/v1",
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
                self.get_response()
        except Exception as e:
            self.get_logger().error('cv_bridge exception: %s' % e)

    def get_response(self):
        prompt = "Use bounding boxes to locate each object in the image. Output the coordinates of all bounding boxes with the object labels in JSON format. Show only the final JSON output without the ```json```."
        with open("photo.png", "rb") as image_file:
            base64_image = base64.b64encode(image_file.read()).decode("utf-8")

        completion = self.client.chat.completions.create(
            #model="gpt-4o-mini",
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

        self.result = completion.choices[0].message.content
        self.get_logger().info(self.result)
        self.update_photo()

    def update_photo(self):
        data = json.loads(self.result)
        objects = data["objects"]
        image = cv2.imread("photo.png")

        for obj in objects:
            bbox = obj['bbox_2d']
            label = obj['label']
            
            x_min, y_min, x_max, y_max = bbox
            color = (255, 0, 0)
            
            # Draw rectangle
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, 2)
            
            # Put label text above the bounding box
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image, label, (x_min, y_min - 10), font, 0.5, color, 2)

        # Save and display output
        cv2.imwrite('output_photo.png', image)
        cv2.imshow('Annotated Image', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)

    object_detection = ObjectDetection()

    rclpy.spin(object_detection)

    object_detection.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
