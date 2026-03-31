"""ROS2 node for downscaling compressed board images.

Subscribes to the high-resolution board image stream, applies a configurable
downsampling method, and republishes the result as a compressed image.
Useful for experimenting with different interpolation methods and optional
color quantization before feeding images to downstream consumers.
"""

import cProfile

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

from .debug import print_profiling_stats


# Map human-readable names to OpenCV interpolation flags
INTERPOLATION_METHODS = {
    "nearest": cv2.INTER_NEAREST,
    "linear": cv2.INTER_LINEAR,
    "area": cv2.INTER_AREA,
    "cubic": cv2.INTER_CUBIC,
    "lanczos": cv2.INTER_LANCZOS4,
}


class ImageDownscalerNode(Node):
    """ROS2 node that subscribes to a compressed image, downscales it, and republishes."""

    def __init__(self):
        """Initialize the ImageDownscalerNode."""
        super().__init__("image_downscaler")

        # ── Topics ──────────────────────────────────────────────────────
        self.declare_parameter("input_image_topic", "board_image/compressed")
        self.input_image_topic = str(self.get_parameter("input_image_topic").value)

        self.declare_parameter("output_image_topic", "board_image_downscaled/compressed")
        self.output_image_topic = str(self.get_parameter("output_image_topic").value)

        # ── Downscaling settings ────────────────────────────────────────
        self.declare_parameter("enable_downscaling", True)
        self.enable_downscaling = bool(self.get_parameter("enable_downscaling").value)

        # Downscale method: "scale", "width", "height", or "exact"
        #   scale  – use scale_factor (0.0–1.0) for both dimensions
        #   width  – resize to target_width, keep aspect ratio
        #   height – resize to target_height, keep aspect ratio
        #   exact  – resize to target_width × target_height (may change aspect ratio)
        self.declare_parameter("downscale_method", "scale")
        self.downscale_method = str(self.get_parameter("downscale_method").value)

        # Scale factor (0.0 – 1.0). Only used when downscale_method="scale".
        self.declare_parameter("scale_factor", 0.5)
        self.scale_factor = float(self.get_parameter("scale_factor").value)

        # Target dimensions. Used by "width", "height", and "exact" methods.
        self.declare_parameter("target_width", 320)
        self.target_width = int(self.get_parameter("target_width").value)

        self.declare_parameter("target_height", 240)
        self.target_height = int(self.get_parameter("target_height").value)

        # Interpolation method: nearest, linear, area, cubic, lanczos
        self.declare_parameter("interpolation", "area")
        self.interpolation_name = str(self.get_parameter("interpolation").value)

        # Validate downscale method
        valid_methods = ("scale", "width", "height", "exact")
        if self.downscale_method not in valid_methods:
            self.get_logger().warn(
                f"Unknown downscale_method '{self.downscale_method}', falling back to 'scale'. "
                f"Valid options: {valid_methods}"
            )
            self.downscale_method = "scale"

        # ── Color quantization (optional) ───────────────────────────────
        # Enable color quantization to reduce the number of unique colors.
        self.declare_parameter("enable_color_quantization", False)
        self.enable_color_quantization = bool(self.get_parameter("enable_color_quantization").value)

        # Number of colors to keep when quantizing (k-means clusters).
        self.declare_parameter("num_colors", 8)
        self.num_colors = int(self.get_parameter("num_colors").value)

        # K-means iterations for color quantization.
        self.declare_parameter("kmeans_iterations", 10)
        self.kmeans_iterations = int(self.get_parameter("kmeans_iterations").value)

        # ── Bit-depth reduction (fast alternative to k-means) ───────────
        # Reduce color depth by zeroing out the N least-significant bits per
        # channel.  e.g. bits_to_strip=4 gives 16 levels per channel.
        self.declare_parameter("enable_bit_strip", False)
        self.enable_bit_strip = bool(self.get_parameter("enable_bit_strip").value)

        self.declare_parameter("bits_to_strip", 4)
        self.bits_to_strip = int(self.get_parameter("bits_to_strip").value)

        # ── Saturation removal (HSV) ────────────────────────────────────
        # Convert to HSV, zero out the S channel, convert back to BGR.
        # This removes color information, leaving only hue-tinted luminance.
        self.declare_parameter("enable_discard_saturation", False)
        self.enable_discard_saturation = bool(self.get_parameter("enable_discard_saturation").value)

        # ── JPEG compression quality (1-100) ────────────────────────────
        self.declare_parameter("jpeg_quality", 80)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        # ── Logging / profiling ─────────────────────────────────────────
        self.declare_parameter("log_fps", True)
        self.log_fps = bool(self.get_parameter("log_fps").value)

        self.declare_parameter("log_fps_interval", 5.0)
        self.log_fps_interval = float(self.get_parameter("log_fps_interval").value)

        # ── Profiling settings ──────────────────────────────────────────
        self.declare_parameter("enable_profiling", False)
        self.enable_profiling = bool(self.get_parameter("enable_profiling").value)

        self.declare_parameter("profiling_duration", 100.0)  # Run profiler for N seconds
        self.profiling_duration = float(self.get_parameter("profiling_duration").value)

        self.declare_parameter("profiling_top_functions", 30)  # Show top N functions in stats
        self.profiling_top_functions = int(self.get_parameter("profiling_top_functions").value)

        # ── Resolve interpolation flag ──────────────────────────────────
        if self.interpolation_name not in INTERPOLATION_METHODS:
            self.get_logger().warn(
                f"Unknown interpolation '{self.interpolation_name}', falling back to 'area'. "
                f"Valid options: {list(INTERPOLATION_METHODS.keys())}"
            )
            self.interpolation_name = "area"
        self.interpolation_flag = INTERPOLATION_METHODS[self.interpolation_name]

        # ── CV Bridge ───────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Publisher ───────────────────────────────────────────────────
        self.publisher = self.create_publisher(CompressedImage, self.output_image_topic, 10)

        # ── Subscriber ──────────────────────────────────────────────────
        self.subscription = self.create_subscription(CompressedImage, self.input_image_topic, self._image_callback, 10)

        # ── FPS tracking ────────────────────────────────────────────────
        self._frame_count = 0
        self._last_fps_time = time.time()

        # ── Bit-depth mask (precompute) ─────────────────────────────────
        self._bit_mask = np.uint8(0xFF << self.bits_to_strip) if self.enable_bit_strip else None

        self.get_logger().info(
            f"Image downscaler started: {self.input_image_topic} → {self.output_image_topic} "
            f"(scale={self.scale_factor}, interp={self.interpolation_name}, "
            f"color_quant={self.enable_color_quantization}, bit_strip={self.enable_bit_strip})"
        )

        # ── Profiling setup ─────────────────────────────────────────────
        self.profiler = None
        if self.enable_profiling:
            self.profiler = cProfile.Profile()
            self.profiler.enable()
            self.get_logger().info(f"cProfile profiling enabled for {self.profiling_duration} seconds")
            # Create one-shot timer to stop profiling after duration
            self.profiling_timer = self.create_timer(self.profiling_duration, self._stop_profiling)

    # ── Callbacks ───────────────────────────────────────────────────────

    def _image_callback(self, msg: CompressedImage) -> None:
        """Receive a compressed image, downscale it, and republish."""
        try:
            # Decompress
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Downscale
            if self.enable_downscaling:
                cv_image = self._downscale(cv_image)

            # Optional color flattening
            if self.enable_color_quantization:
                cv_image = self._quantize_colors(cv_image)

            if self.enable_bit_strip and self._bit_mask is not None:
                cv_image = self._strip_bits(cv_image)

            if self.enable_discard_saturation:
                cv_image = self._discard_saturation(cv_image)

            # Re-compress and publish
            self._publish(cv_image, msg.header)

            # FPS logging
            if self.log_fps:
                self._update_fps()

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    # ── Processing helpers ──────────────────────────────────────────────

    def _downscale(self, image: np.ndarray) -> np.ndarray:
        """Downscale the image according to the configured method.

        Methods:
            scale  - multiply both dimensions by scale_factor.
            width  - resize to target_width, compute height to keep aspect ratio.
            height - resize to target_height, compute width to keep aspect ratio.
            exact  - resize to target_width x target_height (ignores aspect ratio).
        """
        src_h, src_w = image.shape[:2]

        if self.downscale_method == "scale":
            if self.scale_factor >= 1.0:
                return image
            new_width = max(1, int(np.rint(src_w * self.scale_factor).astype(int)))
            new_height = max(1, int(np.rint(src_h * self.scale_factor).astype(int)))

        elif self.downscale_method == "width":
            new_width = self.target_width
            new_height = max(1, int(np.rint(src_h * (new_width / src_w)).astype(int)))

        elif self.downscale_method == "height":
            new_height = self.target_height
            new_width = max(1, int(np.rint(src_w * (new_height / src_h)).astype(int)))
        elif self.downscale_method == "exact":
            new_width = self.target_width
            new_height = self.target_height

        else:
            return image

        if new_width == src_w and new_height == src_h:
            return image

        return cv2.resize(image, (new_width, new_height), interpolation=self.interpolation_flag)

    def _quantize_colors(self, image: np.ndarray) -> np.ndarray:
        """Reduce the number of unique colors via k-means clustering."""
        h, w = image.shape[:2]
        pixels = image.reshape(-1, 3).astype(np.float32)

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, self.kmeans_iterations, 1.0)
        _, labels, centers = cv2.kmeans(pixels, self.num_colors, None, criteria, 1, cv2.KMEANS_PP_CENTERS)

        quantized = centers[labels.flatten()].astype(np.uint8)
        return quantized.reshape(h, w, 3)

    def _strip_bits(self, image: np.ndarray) -> np.ndarray:
        """Reduce color depth by masking out least-significant bits."""
        return cv2.bitwise_and(image, self._bit_mask)

    def _discard_saturation(self, image: np.ndarray) -> np.ndarray:
        """Extract H and V channels from HSV and stack them side-by-side.

        Produces a single-channel (grayscale) image of shape (h, w*2) where
        the left half is the H channel and the right half is the V channel.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h_channel = hsv[:, :, 0]  # Hue
        v_channel = hsv[:, :, 2]  # Value
        return np.hstack((h_channel, v_channel))

    # ── Publishing ──────────────────────────────────────────────────────

    def _publish(self, image: np.ndarray, header) -> None:
        """Publish the transformed and cropped board image as a compressed ROS2 message."""
        try:
            # Convert OpenCV image to ROS CompressedImage message
            msg = self.bridge.cv2_to_compressed_imgmsg(image, dst_format="jpg")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_frame"
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish board image: {e}")

    # ── Profiling ───────────────────────────────────────────────────────

    def _stop_profiling(self) -> None:
        """Stop profiling and print stats (called once by timer)."""
        if self.profiler is not None:
            self.profiler.disable()
            self.get_logger().info("Profiling complete. Generating stats...")
            print_profiling_stats(self.profiler, self.profiling_top_functions)
            self.profiler = None
            # Cancel the timer so it doesn't fire again
            self.profiling_timer.cancel()
            self.profiling_timer = None

    # ── FPS tracking ────────────────────────────────────────────────────

    def _update_fps(self) -> None:
        """Log throughput at the configured interval."""
        self._frame_count += 1
        now = time.time()
        elapsed = now - self._last_fps_time
        if elapsed >= self.log_fps_interval:
            fps = self._frame_count / elapsed
            self.get_logger().info(f"Downscaler throughput: {fps:.1f} FPS")
            self._frame_count = 0
            self._last_fps_time = now


def main(args=None):
    """Run the image downscaler node."""
    rclpy.init(args=args)
    node = ImageDownscalerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
