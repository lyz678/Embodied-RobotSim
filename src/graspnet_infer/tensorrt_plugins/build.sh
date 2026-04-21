#!/bin/bash
# Build script for FPS TensorRT plugin

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

echo "Building FPS TensorRT Plugin..."
echo "Script directory: ${SCRIPT_DIR}"

# Detect TensorRT path
if [ -z "$TENSORRT_ROOT" ]; then
    # Try to auto-detect from common locations
    # Method 1: Check /usr/local/TensorRT-* directories
    if [ -d "/usr/local" ]; then
        TENSORRT_DIRS=$(ls -d /usr/local/TensorRT-* 2>/dev/null | head -1)
        if [ -n "$TENSORRT_DIRS" ] && [ -d "$TENSORRT_DIRS" ]; then
            # Check if it's a direct TensorRT installation or nested
            if [ -d "$TENSORRT_DIRS/include" ] && [ -d "$TENSORRT_DIRS/lib" ]; then
                export TENSORRT_ROOT="$TENSORRT_DIRS"
                echo "Auto-detected TensorRT: ${TENSORRT_ROOT}"
            elif [ -d "$TENSORRT_DIRS/TensorRT-"* ]; then
                # Nested structure like TensorRT-10.14.1.48.Linux.x86_64-gnu.cuda-13.0/TensorRT-10.14.1.48
                NESTED_DIR=$(ls -d "$TENSORRT_DIRS"/TensorRT-* 2>/dev/null | head -1)
                if [ -n "$NESTED_DIR" ] && [ -d "$NESTED_DIR/include" ] && [ -d "$NESTED_DIR/lib" ]; then
                    export TENSORRT_ROOT="$NESTED_DIR"
                    echo "Auto-detected TensorRT (nested): ${TENSORRT_ROOT}"
                fi
            fi
        fi
    fi
    
    # Method 2: Try to find from trtexec path
    if [ -z "$TENSORRT_ROOT" ] && command -v trtexec &> /dev/null; then
        TRTEXEC_PATH=$(which trtexec)
        # Extract TensorRT root from trtexec path
        # e.g., /usr/local/TensorRT-10.14.1.48.Linux.x86_64-gnu.cuda-13.0/TensorRT-10.14.1.48/bin/trtexec
        TRT_BIN_DIR=$(dirname "$TRTEXEC_PATH")
        TRT_ROOT_CANDIDATE=$(dirname "$TRT_BIN_DIR")
        if [ -d "$TRT_ROOT_CANDIDATE/include" ] && [ -d "$TRT_ROOT_CANDIDATE/lib" ]; then
            export TENSORRT_ROOT="$TRT_ROOT_CANDIDATE"
            echo "Auto-detected TensorRT from trtexec: ${TENSORRT_ROOT}"
        fi
    fi
fi

if [ -z "$TENSORRT_ROOT" ]; then
    echo "ERROR: TENSORRT_ROOT not set and cannot auto-detect."
    echo "Please set TENSORRT_ROOT environment variable:"
    echo "  export TENSORRT_ROOT=/usr/local/TensorRT-10.14.1.48.Linux.x86_64-gnu.cuda-13.0/TensorRT-10.14.1.48"
    exit 1
fi

echo "Using TensorRT: ${TENSORRT_ROOT}"

# Create build directory
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# Run CMake
cmake .. -DTensorRT_ROOT="${TENSORRT_ROOT}"

# Build
make -j$(nproc)

if [ -f "libfps_plugin.so" ]; then
    echo ""
    echo "✓ Build successful!"
    echo "Plugin library: ${BUILD_DIR}/libfps_plugin.so"
    ls -lh libfps_plugin.so
else
    echo ""
    echo "✗ Build failed - libfps_plugin.so not found"
    exit 1
fi
