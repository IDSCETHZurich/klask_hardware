"""Main entry point for the Klask state estimation node."""

import rclpy
from .ball_peg_detection import BallPegDetection


def main(args=None):
    rclpy.init(args=args)
    ball_peg_detection = BallPegDetection()
    rclpy.spin(ball_peg_detection)
    ball_peg_detection.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
