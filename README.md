# KLASK Robotic Hardware Stack

Complete KLASK robotic hardware stack, including ROS2 nodes, CAD files, and assembly instructions.

**This README is under construction**

## Prerequisites

- Git, git-lfs, and Docker installed
- User added to the docker group
- X11 forwarding enabled: `xhost +local:root`

## Initial Setup

### Git Configuration

```bash
git config commit.template .gitmessage.txt
```

### SSH Key Setup (for idsc4gpu)

1. Generate SSH key (replace `alex` with your username):

   ```bash
   ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_alex -C "comment"
   ```

2. Add `id_ed25519_alex.pub` content to your GitHub account

3. Edit `~/.ssh/config`:

   ```txt
   Host github.com-alex
       HostName github.com
       User git
       IdentityFile ~/.ssh/id_ed25519_alex
       IdentitiesOnly yes
   ```

4. Clone using the alias:

   ```bash
   git clone git@github.com-alex:IDSCETHZurich/klask_hardware.git
   ```

5. Set local git config:

   ```bash
   git config user.name "Your Name"
   git config user.email "you@example.com"
   ```

## SDK Docker Environment

Build and run the SDK container:

```bash
./ros/ros_env/klask_docker_sdk.sh build
./ros/ros_env/klask_docker_sdk.sh run
```

## VS Code Development

- Click "Reopen in Container" when prompted, or
- Use Remote Containers: Attach to running container from the bottom left menu
