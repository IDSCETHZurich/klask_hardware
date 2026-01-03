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
