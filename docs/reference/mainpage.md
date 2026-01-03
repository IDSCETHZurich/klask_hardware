# KLASK Hardware API Reference

This is the API documentation for the KLASK Hardware ROS2 packages.

## Packages

### klask_motor_commander_pkg

C++ package for controlling ODrive motors in a differential drive configuration. Handles peg synchronization, collision detection, and coordinated movement.

**Key Classes:**

- `Player` - Main controller for player peg movement
- `ODriveController` - Low-level motor communication interface
- `OpenLoopController` - Open-loop control implementation

### klask_imaging_pkg

Python package for camera-based vision processing. Performs perspective transformation, board detection and cropping.

**Key Modules:**

- `CameraNode` - Main ROS2 node for image acquisition and processing
- `utils` - Calibration and filtering utilities
- `debug` - Visualization and debugging tools

### klask_interfaces

Message, service, and action definitions for inter-node communication.

## Getting Started

Navigate through the **Namespaces**, **Classes**, and **Files** tabs above to explore the API documentation.
