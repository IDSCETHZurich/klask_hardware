# Development Environment Setup

This tutorial will guide you through setting up your development environment for working with KLASK Hardware.

## Prerequisites

You need a fully assembled KLASK Hardware system as described in the [Hardware Assembly Tutorial](01-hardware-assembly.md). Additionally, ensure that you are also able to run the Runtime-Container as described in the [Runtime Startup Tutorial](02-runtime-startup.md). To clone the repository you need to have [Git](https://git-scm.com/) and [Git LFS](https://git-lfs.com/) installed on your machine. The only additional software you need is an IDE. We recommend using [Visual Studio Code](https://code.visualstudio.com/) since we provide a ready-to-use configuration for it but you can use any IDE of your choice.

## TODO: FINISH THIS SECTION

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
