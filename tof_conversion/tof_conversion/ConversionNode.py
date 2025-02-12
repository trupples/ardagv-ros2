import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

# LINKS:
#   - https://index.ros.org/p/cv_bridge/


class ConversionNode(Node):
    def __init__(self):
        super().__init__("conversion_node")
        self.declare_parameter("in_topic", value="/cam1/depth_image")
        self.declare_parameter("out_topic", value="/scan")

        in_topic_name = self.get_parameter("in_topic").get_parameter_value().string_value
        out_topic_name = self.get_parameter("out_topic").get_parameter_value().string_value

        # Create a subscriber
        self.get_logger().info(f"Subscribing to topic: {in_topic_name}")
        self.subscriber_ = self.create_subscription(
            Image, in_topic_name, self.image_transform_callback, qos_profile=10
        )

        # Create a publisher
        self.get_logger().info(f"Publishing to topic: {out_topic_name}")
        self.publisher = self.create_publisher(Image, out_topic_name, 10)

        # Bridge for converting between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

    def image_transform_callback(self, msg: Image):
        # data = np.array(msg.data).reshape((msg.height, msg.width))
        # self.get_logger().info(f"Received: {data.shape}")
        self.get_logger().info(f"Height: {msg.height}")
        self.get_logger().info(f"width: {msg.width}")
        self.get_logger().info(f"encoding: {msg.encoding}")
        self.get_logger().info(f"is_bigendian: {msg.is_bigendian}")
        self.get_logger().info(f"step: {msg.step}")

        # Convert the ROS Image message to a cv2 image for processing
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding=msg.encoding)
            self.get_logger().info(f"Converted to cv2 image: {cv_image.shape}")

            # # Convert the image to grayscale
            # gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # self.get_logger().info(f"Converted to grayscale: {gray_image.shape}")

            # # Convert the grayscale image to a ROS Image message
            # ros_image = self.bridge.cv2_to_imgmsg(gray_image, "mono8")
            # self.get_logger().info(f"Converted to ROS Image: {ros_image.height}x{ros_image.width}")

            # # Publish the ROS Image message
            # self.publisher.publish(ros_image)
        except CvBridgeError as e:
            self.get_logger().error(f"Error: {e}")

        # Convert the cv2 image to a ROS LaserScan message
        # laser_scan = self.bridge.cv2_to_imgmsg(cv_image, "mono8")


def main(args=None):
    rclpy.init(args=args)

    node = ConversionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
