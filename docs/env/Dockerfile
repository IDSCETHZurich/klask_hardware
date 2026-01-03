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

# Create doxygen build script
RUN echo '#!/bin/bash\n\
set -e\n\
echo "Building Doxygen documentation..."\n\
mkdir -p docs/reference/api/doxygen\n\
doxygen docs/Doxyfile\n\
echo "✓ Doxygen build complete"\n\
' > /usr/local/bin/build-doxygen.sh && \
    chmod +x /usr/local/bin/build-doxygen.sh

# Create mkdocs build script
RUN echo '#!/bin/bash\n\
set -e\n\
echo "Building MkDocs documentation..."\n\
mkdocs build --strict -f docs/mkdocs.yml\n\
echo "✓ MkDocs build complete"\n\
' > /usr/local/bin/build-mkdocs.sh && \
    chmod +x /usr/local/bin/build-mkdocs.sh

# Create combined build script
RUN echo '#!/bin/bash\n\
set -e\n\
/usr/local/bin/build-doxygen.sh\n\
echo ""\n\
/usr/local/bin/build-mkdocs.sh\n\
' > /usr/local/bin/build-all.sh && \
    chmod +x /usr/local/bin/build-all.sh

# Default command
CMD ["/usr/local/bin/build-all.sh"]
