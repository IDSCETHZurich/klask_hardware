#!/usr/bin/env python3
"""
Simple script to check the calibration status of the motor commander system.
"""

import rclpy
from rclpy.node import Node
from klask_interfaces.srv import GetCalibrationStatus
import sys


def check_calibration_status():
    """Check and print the calibration status."""
    rclpy.init()
    node = Node('calibration_status_checker')
    
    client = node.create_client(GetCalibrationStatus, 'get_calibration_status')
    
    node.get_logger().info('Waiting for calibration status service...')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('Service not available')
        node.destroy_node()
        rclpy.shutdown()
        return False
    
    request = GetCalibrationStatus.Request()
    future = client.call_async(request)
    
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    
    if future.done():
        response = future.result()
        node.get_logger().info(f'Calibration status: {response.is_calibrated}')
        node.get_logger().info(f'Message: {response.message}')
        
        node.destroy_node()
        rclpy.shutdown()
        return response.is_calibrated
    else:
        node.get_logger().error('Service call failed')
        node.destroy_node()
        rclpy.shutdown()
        return False


if __name__ == '__main__':
    is_calibrated = check_calibration_status()
    sys.exit(0 if is_calibrated else 1)
