# Camera Node Configuration Guide

This guide explains how to configure and launch the camera node with custom parameters.

## Quick Start

### Launch with default parameters:
```bash
ros2 launch klask_imaging_pkg camera_launch.py
```

### Launch with custom parameter file:
```bash
ros2 launch klask_imaging_pkg camera_launch.py params_file:=/path/to/custom_params.yaml
```

### Run node directly (no parameters):
```bash
ros2 run klask_imaging_pkg camera_node
```

## Parameter Configuration

All configurable parameters are documented in `config/camera_node_params.yaml`. Copy this file and modify values as needed for your setup.

### Parameter Categories

#### Camera Configuration
- `camera_fps`: Camera frame rate (default: 120 Hz)
- `camera_width`: Resolution width (default: 1280 pixels)
- `camera_height`: Resolution height (default: 720 pixels)

#### Detection Thresholds
- `flood_threshold`: Board boundary detection sensitivity (default: 120)
- `goal_flood_threshold`: Goal detection sensitivity (default: 20)

#### Border Detection
- `boarder_seg_inside_offset`: Inside offset for border region (default: 30 pixels)
- `boarder_seg_outside_offset`: Outside offset for border region (default: 20 pixels)
- `boarder_seg_corner_distance`: Corner exclusion distance (default: 100 pixels)

#### Goal Detection
- `goal_h_factor`: Horizontal position factor (default: 0.085)
- `goal_o_factor`: Offset factor for seed points (default: 0.01)
- `goal_e_factor`: Ellipse size scaling (default: 1.2)

#### Image Processing
- `use_smoothing`: Enable Gaussian blur (default: true)
- `smoothing_kernel`: Blur kernel size (default: 5, must be odd)

#### Line Validation
- `line_angle_threshold`: Max angle change between frames (default: 10.0 degrees)
- `line_position_threshold`: Max position change (default: 50.0 pixels)
- `min_edge_points`: Min points for line fitting (default: 10)

#### Debug/Display
- `debug_view`: Enable debug visualizations (default: false)
- `show_image`: Display output image (default: false)
- `show_image_fps`: Display update rate (default: 10 Hz)

#### Profiling
- `enable_profiling`: Enable performance profiling (default: false)
- `profiling_duration`: Profiling duration (default: 100.0 seconds)
- `profiling_top_functions`: Number of functions to show (default: 30)

## Runtime Parameter Changes

You can view and modify parameters while the node is running:

### List all parameters:
```bash
ros2 param list /camera_node
```

### Get a specific parameter:
```bash
ros2 param get /camera_node flood_threshold
```

### Set a parameter at runtime:
```bash
ros2 param set /camera_node flood_threshold 150
```

### Save current parameters to file:
```bash
ros2 param dump /camera_node --output-dir ./
```

## Troubleshooting

### Board not detected properly
- Increase `flood_threshold` if detecting too much area
- Decrease `flood_threshold` if detecting too little
- Enable `debug_view: true` to visualize detection steps

### Goals not detected
- Adjust `goal_flood_threshold` (lower = more sensitive)
- Modify `goal_h_factor` to change horizontal search position
- Check goal position in debug view

### Unstable border detection
- Enable smoothing: `use_smoothing: true`
- Increase `smoothing_kernel` (5, 7, 9)
- Adjust `line_angle_threshold` and `line_position_threshold` for outlier rejection

### Camera issues
- Verify `camera_fps`, `camera_width`, `camera_height` match your hardware
- Check camera permissions and USB connection
- Review node logs for camera configuration messages

## Example Custom Configuration

Create a file `my_camera_params.yaml`:
```yaml
/**:
  ros__parameters:
    camera_fps: 60
    camera_width: 1920
    camera_height: 1080
    flood_threshold: 100
    debug_view: true
    show_image: true
```

Launch with custom config:
```bash
ros2 launch klask_imaging_pkg camera_launch.py params_file:=$(pwd)/my_camera_params.yaml
```

## Contributing

When adding new configurable parameters:
1. Declare parameter in `camera_node.py` `__init__()` with `self.declare_parameter()`
2. Get parameter value with `self.get_parameter().value`
3. Add parameter to `camera_node_params.yaml` with documentation
4. Update this README with parameter description
