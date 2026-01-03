# Development Environment Setup

This tutorial will guide you through setting up your development environment for working with KLASK Hardware.

## Prerequisites

## TODO: FINISH THIS SECTION

## Documentation

The documentation for KLASK Hardware is built using [MkDocs](https://www.mkdocs.org/) with the [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme. The docs are automatically built and deployed using GitHub Actions upon changes to the `main` branch. To preview the documentation locally, we provide a Docker setup in `docs/env/`

To build the documentation locally just run the following command from the root of the repository:

```bash
./docs/env/build_docs.sh --build
```

On the first call this will build the Docker image and then build the documentation. On subsequent calls it will only build the documentation. The generated output will be available in the `site/` folder. Just open the `index.html` file in your browser to view the documentation.

Make sure to check the output for any warnings or errors that need to be addressed before pushing your changes.
