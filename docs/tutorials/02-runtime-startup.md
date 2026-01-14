# Runtime Startup

Now that you have assembled the KLASK Hardware system as described in the [Hardware Assembly Tutorial](01-hardware-assembly.md), the next step is to set up and run the Runtime-Container which provides the necessary software environment for operating the KLASK system.

If you simply want to run the prebuilt inference node to play against the robot or let the robot play against itself, you can have a look at the startup instructions of the [klask_software repository](https://github.com/IDSCETHZurich/klask_software). These commands will pull and run all the stuff from the [klask_hardware repository](https://github.com/IDSCETHZurich/klask_hardware) automatically.

But if you want to start up the ROS packages used to interact with the hardware to test your own agent or modify the software, you have serval options to choose from.

## 1. Using the Prebuilt Runtime Container

The easiest way to get started is to use the prebuilt Runtime Container that we provide. This container includes all necessary dependencies and is ready to use out of the box.

The only requirement is that you have [Docker](https://www.docker.com/get-started/) installed on your system.

Simply download the [`ros/ros_env/klask_prebuilt_runtime.sh`](https://github.com/IDSCETHZurich/klask_hardware/blob/main/ros/ros_env/klask_prebuilt_runtime.sh) script from this repo, which wraps some useful Docker commands to run the prebuilt container. Once you have the script, make sure it is executable:

```bash
chmod +x klask_prebuilt_runtime.sh
```

Then you can run the container with:

```bash
./klask_prebuilt_runtime.sh
```

This will pull the latest prebuilt Runtime Container from our GitHub Container Registry and start it up and launch the necessary ROS nodes to interact with the KLASK hardware.

The script also supports additional options:

```bash
# Run latest release
klask_prebuilt_runtime.sh

# Run specific version
klask_prebuilt_runtime.sh --tag v1.0.0

# Pull latest before running
klask_prebuilt_runtime.sh pull

# Connect to the running container via bash
klask_prebuilt_runtime.sh connect

# Stop the running container
klask_prebuilt_runtime.sh stop
```

You can additionally pass ROS arguments to specify if you want to launch the left, right, or both sides of the KLASK robot and whether you want to launch the image viewer which displays the camera feed:

```bash
# The default is to launch both players with viewer
klask_prebuilt_runtime.sh

# Launch right player without viewer
klask_prebuilt_runtime.sh --player right --no-viewer
```

All the available options can be listed with:

```bash
klask_prebuilt_runtime.sh --help
```

## 2. Building the Container Locally

If you have a development setup as described in the [Development Setup Tutorial](03-dev-setup.md), you can also build the Runtime Container locally from source. This allows you to modify the source code and test your changes directly within the container. Details on how to build and run the container can be found in the [Development Setup Tutorial](03-dev-setup.md).
