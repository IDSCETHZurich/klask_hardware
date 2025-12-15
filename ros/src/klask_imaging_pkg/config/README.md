# Camera Node Configuration Guide

This guide explains how to configure and launch the camera node with custom parameters.

## Quick Start

### Launch with default parameters

```bash
ros2 launch klask_imaging_pkg camera_launch.py
```

or

```bash
ros2 run klask_imaging_pkg camera_node
```

### Launch with custom parameter file

```bash
ros2 launch klask_imaging_pkg camera_launch.py params_file:=/path/to/custom_params.yaml
```

## Parameter Configuration

All configurable parameters are documented in `config/camera_node_params.yaml`. Copy this file and modify values as needed for your setup.
