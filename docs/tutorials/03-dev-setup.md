# Development Environment Setup

This tutorial will guide you through setting up your development environment for working with KLASK Hardware.

## Prerequisites

You need a fully assembled KLASK Hardware system as described in the [Hardware Assembly Tutorial](01-hardware-assembly.md). Additionally, ensure that you are also able to run the Runtime-Container as described in the [Runtime Startup Tutorial](02-runtime-startup.md). To clone the repository you need to have [Git](https://git-scm.com/) and [Git LFS](https://git-lfs.com/) installed on your machine. The only additional software you need is an IDE. We recommend using [Visual Studio Code](https://code.visualstudio.com/) since we provide a ready-to-use configuration for it but you can use any IDE of your choice.

## Cloning the Repository

First, clone the `klask_hardware` repository to your local machine. If you are an IDSC student working on the idsc4gpu machine, please refer to the [Additional Resources for IDSC Students](#additional-resources-for-idsc-students) section at the end of this tutorial for hints.

All you need to develop on the KLASK Hardware is provided in the docker containers so you don't need to install any additional dependencies on your host machine.

The script `./ros/ros_env/klask_docker_helper.sh` warps common docker commands to make it easier to work with the containers and the `.devcontainer/devcontainer.json` file provides a ready-to-use configuration for Visual Studio Code.

There are two main containers you will be working with:

- **SDK Container**: Used for development, building, and testing code. This container includes all necessary development and debugging tools.
- **Runtime Container**: Used for running the KLASK Hardware system. This container is optimized for performance and includes only the necessary runtime components.

## Working with the Containers

### Building the Containers

Before you can start developing, you need to build the container. From the root of the repository, run:

```bash
# Build the SDK container
./ros/ros_env/klask_docker_helper.sh build

# (Optional) Build the Runtime container
./ros/ros_env/klask_docker_helper.sh -r build
```

### Running the Containers

When working with the SDK container, you have two main options:

#### 1. Using Visual Studio Code Dev Containers

If you are using Visual Studio Code, you can open the repository folder and use the Dev Containers feature to open the project inside the SDK container. This provides a seamless development experience with access to all tools and extensions configured in the container.

1. Open Visual Studio Code.
2. Open the `klask_hardware` folder.
3. You should see a prompt to reopen the folder in a container. Click **"Reopen in Container"**.
4. Visual Studio Code will build and start the SDK container if it is not already running, and open the project inside it.
5. You can now use the integrated terminal, debugger, and other features of Visual Studio Code within the container environment. We already configured debugging configurations so that you can directly start debugging the different components of the KLASK Hardware system.

If the reopen prompt does not appear, you can manually open the command palette (Ctrl+Shift+P or Cmd+Shift+P) and select **"Dev Containers: Reopen in Container"** or click on the bottom-left icon "><" to open the "Remote Window" menu and select **"Reopen in Container"**.

#### 2. Using the Command Line

You can also run the SDK container directly from the command line using the helper script. This is useful if you prefer working in a terminal or are using a different IDE.

```bash
# Run the SDK container
./ros/ros_env/klask_docker_helper.sh run
```

Now you can connect your IDE to the running container or use the terminal inside the container for development with:

```bash
# Connect to the running SDK container
./ros/ros_env/klask_docker_helper.sh connect
```

If you want to shutdown the container, you can do so with:

```bash
# Stop the SDK container
./ros/ros_env/klask_docker_helper.sh stop
```

The helper script also provides other usefull commands. You can see all available options with:

```bash
./ros/ros_env/klask_docker_helper.sh --help
```

#### 3. Working with the Runtime Container

Since the Runtime container is optimized for performance and does not include development tools, it is typically only used for running the KLASK Hardware system. You can start the Runtime container using the helper script:

```bash
# Run the Runtime container
./ros/ros_env/klask_docker_helper.sh -r run
```

All the commands from the SDK container helper also work for the Runtime container by adding the `-r` flag. Additionally, you can pass ROS arguments to the container with:

```bash
# Run the left player only (default is both)
./ros/ros_env/klask_docker_helper.sh -r run --player left
# Run the right player only without the image viewer
./ros/ros_env/klask_docker_helper.sh -r run --player right --no-viewer
```

For a list of all available options for the Runtime container, use:

```bash
./ros/ros_env/klask_docker_helper.sh --help
```

## Documentation

The documentation for KLASK Hardware is built using [MkDocs](https://www.mkdocs.org/) with the [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme, and [Doxygen](https://www.doxygen.nl/) for API reference documentation. The docs are automatically built and deployed using GitHub Actions upon changes to the `main` branch. To preview the documentation locally, we provide a Docker setup in `docs/env/`

### Building Documentation

To build the documentation locally, use the `docs.sh` script from the root of the repository:

```bash
# Build all documentation (Doxygen + MkDocs)
./docs/env/docs.sh --build

# Build only Doxygen API documentation (faster for API changes)
./docs/env/docs.sh --doxygen

# Build only MkDocs site (faster for content changes)
./docs/env/docs.sh --mkdocs

# Rebuild the Docker image (if you need to update dependencies)
./docs/env/docs.sh --rebuild-image
```

On the first call this will build the Docker image and then build the documentation. On subsequent calls it will only build the documentation.

### Output Locations

- **MkDocs site**: `site/` folder - open `site/index.html` in your browser
- **Doxygen API docs**: `docs/reference/api/doxygen/` folder - open `docs/reference/api/doxygen/index.html` in your browser

Make sure to check the output for any warnings or errors that need to be addressed before pushing your changes.

## Additional Resources for IDSC Students

If you are an IDSC student working on the idsc4gpu machine using the `student` user you need to setup the ssh key and repo access a bit differently to be able to safely work on the machine and not grant other students full access to your github profile.

1. Generate a new SSH key on the idsc4gpu machine (replace `alex` with your username):

    ```bash
    ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519_alex -C "idsc4gpu-alex"
    ```

    Add a passphrase when prompted to secure your key. This is important since other students have access to the machine.

2. Add `id_ed25519_alex.pub` content to your GitHub account.

3. Edit the `~/.ssh/config` to add an alias for github with your new key:

    ```txt
    Host github.com-alex
        HostName github.com
        User git
        IdentityFile ~/.ssh/id_ed25519_alex
        IdentitiesOnly yes
    ```

4. Clone using the alias. It is important to use the alias `github.com-alex` here to ensure the correct key is used:

    ```bash
    git clone git@github.com-alex:IDSCETHZurich/klask_hardware.git
    ```

5. Set the local git config for your commits:

    ```bash
    cd klask_hardware
    git config user.name "Alex Student"
    git config user.email "alex.student@example.com"
    ```

## Additional Resources for Maintainers

### Creating a New Release

Releases are used to publish versioned runtime containers to GitHub Container Registry (GHCR). When you create a release, a GitHub Actions workflow automatically builds and publishes the runtime container image.

#### Release Process

1. **Navigate to Releases**
    - Go to the GitHub repository
    - Click on **"Releases"** in the right sidebar

2. **Create a New Release**
    - Click **"Draft a new release"**

3. **Choose or Create a Tag**
    - Click **"Choose a tag"**
    - Type a new version tag following semantic versioning (e.g., `v1.0.0`, `v1.2.3`)
    - Click **"Create new tag: vX.Y.Z on publish"**

4. **Add Release Information**
    - **Release title**: Use a descriptive title (e.g., "Release v1.0.0" or "Initial Public Release")
    - **Release description**: Add release notes describing what changed:
        - New features
        - Bug fixes
        - Breaking changes
        - Known issues

5. **Publish the Release**
    - Click **"Publish release"**
    - The GitHub Actions workflow will automatically:
        - Build the runtime container
        - Push it to GHCR with multiple tags:
           - `ghcr.io/idscethzurich/klask_hardware/runtime:latest`
           - `ghcr.io/idscethzurich/klask_hardware/runtime:1.0.0`
           - `ghcr.io/idscethzurich/klask_hardware/runtime:1.0`

#### Version Numbering Guidelines

Follow [Semantic Versioning](https://semver.org/):

- **Major version** (v**X**.0.0): Breaking changes that require user action
- **Minor version** (v1.**X**.0): New features that are backward compatible
- **Patch version** (v1.0.**X**): Bug fixes and minor improvements
