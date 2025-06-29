import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_interface.msg import ObjectPose
import cv2
from PIL import Image as pilImage
from cv_bridge import CvBridge
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")

        self.pub_image = self.create_publisher(Image, "output_image",10) #to publish processed image
        self.pose_pub = self.create_publisher(ObjectPose, "pose_publisher",10) #to publish processed image
        self.sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10) #subscribed to get image data

        self.get_logger().info("Vision node has been started!")
        self.bridge = CvBridge()
        self.centroid_list = [0,0,0,0,0,0]

        # color ranges
        self.lower_blue = np.array([100, 150, 0])
        self.upper_blue = np.array([140, 255, 255])

        self.lower_red_1 = np.array([0, 120, 70])
        self.upper_red_1 = np.array([10, 255, 255])
        self.lower_red_2 = np.array([170, 120, 70])
        self.upper_red_2 = np.array([180, 255, 255])

        self.lower_green = np.array([40, 50, 50])
        self.upper_green = np.array([80, 255, 255])
        # self color list
        self.colors = [(255,0,0),(0,255,0),(0,0,255)]
        

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame,red_x,red_y, green_x, green_y, blue_x, blue_y = self.find_object_centroid(cv_image)
        imgmsg = self.bridge.cv2_to_imgmsg(frame, "bgr8")

        centroids = ObjectPose()
        centroids.object_1 = "Red" 
        centroids.pixel1_x = red_x
        centroids.pixel1_y = red_y
        centroids.object_2 = "Green"
        centroids.pixel2_x = green_x
        centroids.pixel2_y = green_y
        centroids.object_3 = "Blue"
        centroids.pixel3_x = blue_x
        centroids.pixel3_y = blue_y

        self.pose_pub.publish(centroids)
        self.pub_image.publish(imgmsg)
    
    def find_object_centroid(self,og_image):
        # hsv image
        hsv_image = cv2.cvtColor(og_image, cv2.COLOR_BGR2HSV)
        # blue mask
        blue_mask = cv2.inRange(hsv_image, self.lower_blue, self.upper_blue)
        bbox_blue = pilImage.fromarray(blue_mask).getbbox()
        if bbox_blue is not None:
            x1,y1,x2,y2 = bbox_blue
            center_blue_x = (x1 + x2) // 2
            center_blue_y = (y1 + y2) // 2
            cv2.rectangle(og_image, (x1,y1), (x2,y2), (255,0,0), 5)
            cv2.circle(og_image, (center_blue_x, center_blue_y), 5, (255,0,0), 3)
            self.centroid_list[4] = int(center_blue_x)
            self.centroid_list[5] = int(center_blue_y)

        else:
            self.centroid_list[4] = 0
            self.centroid_list[5] = 0
        

        # green mask
        green_mask = cv2.inRange(hsv_image, self.lower_green, self.upper_green)
        bbox_green = pilImage.fromarray(green_mask).getbbox()
        if bbox_green is not None:
            x1,y1,x2,y2 = bbox_green
            center_green_x = (x1 + x2) // 2
            center_green_y = (y1 + y2) // 2
            cv2.rectangle(og_image, (x1,y1), (x2,y2), (0,255,0), 5)
            cv2.circle(og_image, (center_green_x, center_green_y), 5, (0,255,0), 3)
            self.centroid_list[2] = int(center_green_x)
            self.centroid_list[3] = int(center_green_y)

        else:
            self.centroid_list[2] = 0
            self.centroid_list[3] = 0

        # red masks
        red_mask1 = cv2.inRange(hsv_image, self.lower_red_1, self.upper_red_1)
        red_mask2 = cv2.inRange(hsv_image, self.lower_red_2, self.upper_red_2)
        red_mask = red_mask1+red_mask2
        bbox_red = pilImage.fromarray(red_mask).getbbox()
        if bbox_red is not None:
            x1,y1,x2,y2 = bbox_red
            center_red_x = (x1 + x2) // 2
            center_red_y = (y1 + y2) // 2
            cv2.rectangle(og_image, (x1,y1), (x2,y2), (0,0,255), 5)
            cv2.circle(og_image, (center_red_x, center_red_y), 5, (0,0,255), 3)  
            self.centroid_list[0] = int(center_red_x)
            self.centroid_list[1] = int(center_red_y)

        else:
            self.centroid_list[0] = 0
            self.centroid_list[1] = 0

        
        return og_image, self.centroid_list[0], self.centroid_list[1],self.centroid_list[2], self.centroid_list[3], self.centroid_list[4], self.centroid_list[5]



def main(args=None):
    rclpy.init(args=args)
    imageSubscriber = ImageSubscriber()
    rclpy.spin(imageSubscriber)
    imageSubscriber.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()