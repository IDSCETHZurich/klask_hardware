# Development Environment Setup

This tutorial will guide you through setting up your development environment for working with KLASK Hardware.

## Prerequisites

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
