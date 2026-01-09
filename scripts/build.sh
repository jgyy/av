#!/bin/bash

# Build script for AV Simulator

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
BUILD_DIR="build"
BUILD_TYPE="${1:-Release}"
PARALLEL_JOBS=$(nproc)

echo -e "${GREEN}===== AV Simulator Build Script =====${NC}"
echo "Build Type: $BUILD_TYPE"
echo "Build Directory: $BUILD_DIR"
echo "Parallel Jobs: $PARALLEL_JOBS"

# Create build directory
if [ ! -d "$BUILD_DIR" ]; then
    echo -e "${YELLOW}Creating build directory...${NC}"
    mkdir -p "$BUILD_DIR"
fi

# Navigate to build directory
cd "$BUILD_DIR"

# Run CMake
echo -e "${YELLOW}Configuring with CMake...${NC}"
cmake -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
      ..

# Build
echo -e "${YELLOW}Building project...${NC}"
cmake --build . --config "$BUILD_TYPE" --parallel "$PARALLEL_JOBS"

# Run tests if built
if [ -f "tests/test_foundation" ]; then
    echo -e "${YELLOW}Running tests...${NC}"
    ctest --output-on-failure
fi

echo -e "${GREEN}Build complete!${NC}"
echo "Executable: $BUILD_DIR/av_exe"
echo ""
echo "To run the simulator:"
echo "  cd $BUILD_DIR"
echo "  ./av_exe"
