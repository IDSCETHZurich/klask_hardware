# KLASK Robotic Hardware Stack

Complete KLASK robotic hardware stack, including ROS2 nodes, CAD files, and assembly instructions.

## This Section is Under Construction

```bash
git config commit.template .gitmessage.txt
xhost +local:root
```

Prerequisites

- Git and Docker installed
- Your user added to the docker group
- (Docker compose installed)

Repo Setup on `idsc4gpu`

- Create SSH-Key on machine with passphrase (alex is the example user here)

    ```bash
    ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_alex -C "comment"
    ```

- Add the content of the id_ed25519_alex.pub to your github account
- edit the .ssh/config and add your user and newly generated ssh key

    ```bash
    Host github.com-alex
        HostName github.com
        User git
        IdentityFile ~/.ssh/id_ed25519_alex
        IdentitiesOnly yes
    ```

- clone the repo using tha alias you specified in the config file

    ```bash
    git clone git@github.com-alex:IDSCETHZurich/Cubli-Evo_Firmware.git
    ```

- add your user and email to the local git config (open a terminal in the repo folder)

    ```bash
    git config user.name  "Your Name"
    git config user.email "you@example.com"
    ```

SDK:

```bash
./ros/ros_env/klask_docker_sdk.sh build
./ros/ros_env/klask_docker_sdk.sh run
```

VS Code

- bottom left
- attach to running container
- select container

Or just click on reopen in container
