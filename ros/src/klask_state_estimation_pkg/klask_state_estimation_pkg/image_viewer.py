"""ROS2 node for viewing compressed image stream from the board camera."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import time


class ImageViewer(Node):
    """ROS2 node that subscribes to compressed images and displays them."""
    
    def __init__(self):
        super().__init__("image_viewer")
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Subscribe to compressed image topic
        self.image_subscription = self.create_subscription(
            CompressedImage,
            "board_image/compressed",
            self.image_callback,
            10
        )
        
        self.get_logger().info("Image viewer node started. Subscribing to 'board_image/compressed'")
        
        # Window name
        self.window_name = "Board Image Stream"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        # FPS tracking
        self.fps_update_interval = 1.0  # Update FPS display every second
        self.fps_last_update_time = time.time()
        self.fps_frame_count = 0
        self.current_fps = 0.0
    
    def image_callback(self, msg: CompressedImage) -> None:
        """Callback for receiving compressed images."""
        try:
            # Convert ROS CompressedImage message to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Update FPS counter
            self._update_fps()
            
            # Draw FPS on image
            if self.current_fps > 0:
                fps_text = f"FPS: {self.current_fps:.1f}"
                cv2.putText(
                    cv_image,
                    fps_text,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
            
            # Display the image
            cv2.imshow(self.window_name, cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")
    
    def _update_fps(self) -> None:
        """Update FPS calculation."""
        self.fps_frame_count += 1
        current_time = time.time()
        elapsed = current_time - self.fps_last_update_time
        
        if elapsed >= self.fps_update_interval:
            self.current_fps = self.fps_frame_count / elapsed
            self.fps_frame_count = 0
            self.fps_last_update_time = current_time
    
    def destroy_node(self):
        """Clean up when node is destroyed."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    image_viewer = ImageViewer()
    
    try:
        rclpy.spin(image_viewer)
    except KeyboardInterrupt:
        pass
    finally:
        image_viewer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
