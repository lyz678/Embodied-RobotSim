cmake_minimum_required(VERSION 3.13)
project(fps_plugin)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CUDA_STANDARD 14)

# Enable CUDA language support
enable_language(CUDA)

# Find CUDA
# Try to find CUDA toolkit
find_package(CUDAToolkit QUIET)
if(NOT CUDAToolkit_FOUND)
    # Fallback to old FindCUDA
    find_package(CUDA REQUIRED)
    set(CUDAToolkit_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS})
    set(CUDAToolkit_LIBRARY_DIR ${CUDA_LIBRARY_DIRS})
endif()

# Find TensorRT
# Try to find TensorRT from common locations or environment variable
if(DEFINED ENV{TENSORRT_ROOT})
    set(TensorRT_ROOT $ENV{TENSORRT_ROOT})
elseif(EXISTS "/usr/local/TensorRT")
    # Try to find latest TensorRT version
    file(GLOB TENSORRT_DIRS "/usr/local/TensorRT-*")
    if(TENSORRT_DIRS)
        list(SORT TENSORRT_DIRS)
        list(REVERSE TENSORRT_DIRS)
        list(GET TENSORRT_DIRS 0 TensorRT_ROOT)
    endif()
endif()

# If TensorRT_ROOT is set, use it
if(TensorRT_ROOT)
    set(TENSORRT_INCLUDE_DIR "${TensorRT_ROOT}/include")
    set(TENSORRT_LIB_DIR "${TensorRT_ROOT}/lib")
    message(STATUS "Using TensorRT from: ${TensorRT_ROOT}")
else()
    # Try to find TensorRT using find_package
    find_package(TensorRT QUIET)
    if(NOT TensorRT_FOUND)
        message(FATAL_ERROR "TensorRT not found. Please set TENSORRT_ROOT environment variable or install TensorRT.")
    endif()
endif()

# Include directories
include_directories(${CUDAToolkit_INCLUDE_DIRS})
include_directories(${TENSORRT_INCLUDE_DIR})

# CUDA files
set(CUDA_SOURCES
    fps_plugin.cu
)

# C++ files
set(CPP_SOURCES
    fps_plugin.cpp
)

set(CMAKE_CUDA_ARCHITECTURES 89)

# Create library
add_library(fps_plugin SHARED ${CUDA_SOURCES} ${CPP_SOURCES})
set_target_properties(fps_plugin PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
    POSITION_INDEPENDENT_CODE ON
)

# Find TensorRT libraries
if(TensorRT_ROOT)
    find_library(TENSORRT_NVINFER_LIB nvinfer PATHS ${TENSORRT_LIB_DIR} NO_DEFAULT_PATH)
    find_library(TENSORRT_NVINFER_PLUGIN_LIB nvinfer_plugin PATHS ${TENSORRT_LIB_DIR} NO_DEFAULT_PATH)
else()
    set(TENSORRT_NVINFER_LIB ${TENSORRT_LIBRARY})
    set(TENSORRT_NVINFER_PLUGIN_LIB ${TENSORRT_PLUGIN_LIBRARY})
endif()

# Link libraries
target_link_libraries(fps_plugin
    ${TENSORRT_NVINFER_LIB}
    ${TENSORRT_NVINFER_PLUGIN_LIB}
)

# Link CUDA libraries
if(CUDAToolkit_FOUND)
    target_link_libraries(fps_plugin CUDA::cudart)
else()
    # Fallback: find cudart manually
    # Check common CUDA installation paths
    set(CUDA_LIB_PATHS
        ${CUDAToolkit_LIBRARY_DIR}
        ${CUDA_LIBRARY_DIRS}
        /usr/local/cuda-13.1/targets/x86_64-linux/lib
        /usr/local/cuda-13.1/lib64
        /usr/local/cuda-13.1/lib
        /usr/local/cuda/lib64
        /usr/local/cuda/lib
        /usr/lib/x86_64-linux-gnu
    )
    
    find_library(CUDART_LIB 
        NAMES cudart libcudart.so
        PATHS ${CUDA_LIB_PATHS}
    )
    
    if(CUDART_LIB)
        message(STATUS "Found cudart: ${CUDART_LIB}")
        target_link_libraries(fps_plugin ${CUDART_LIB})
    else()
        message(WARNING "cudart not found via find_library, trying direct path")
        # Try direct path
        if(EXISTS "/usr/local/cuda-13.1/targets/x86_64-linux/lib/libcudart.so")
            target_link_libraries(fps_plugin /usr/local/cuda-13.1/targets/x86_64-linux/lib/libcudart.so)
        elseif(EXISTS "/usr/local/cuda-13.1/lib64/libcudart.so")
            target_link_libraries(fps_plugin /usr/local/cuda-13.1/lib64/libcudart.so)
        else()
            # Last resort: add library directory and link
            link_directories(/usr/local/cuda-13.1/targets/x86_64-linux/lib)
            target_link_libraries(fps_plugin cudart)
        endif()
    endif()
endif()

# Set output directory
set_target_properties(fps_plugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
)

# Print configuration info
if(CUDAToolkit_FOUND)
    message(STATUS "CUDAToolkit_INCLUDE_DIRS: ${CUDAToolkit_INCLUDE_DIRS}")
    message(STATUS "CUDAToolkit_LIBRARY_DIR: ${CUDAToolkit_LIBRARY_DIR}")
else()
    message(STATUS "CUDA_INCLUDE_DIRS: ${CUDA_INCLUDE_DIRS}")
    message(STATUS "CUDA_LIBRARY_DIRS: ${CUDA_LIBRARY_DIRS}")
endif()
message(STATUS "TENSORRT_INCLUDE_DIR: ${TENSORRT_INCLUDE_DIR}")
message(STATUS "TENSORRT_LIB_DIR: ${TENSORRT_LIB_DIR}")
