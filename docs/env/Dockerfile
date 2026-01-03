FROM python:3.12-slim

# Install system dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    doxygen \
    graphviz \
    rsync && \
    rm -rf /var/lib/apt/lists/*

# Install Python documentation tools
RUN pip install --no-cache-dir \
    mkdocs \
    mkdocs-material \
    mkdocs-glightbox

# Set working directory
WORKDIR /workspace

# Default command
CMD ["mkdocs", "build", "--strict", "-f", "docs/mkdocs.yml"]
