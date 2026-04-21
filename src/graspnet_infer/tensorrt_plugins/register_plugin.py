"""
Register TensorRT FPS plugin
"""
import tensorrt as trt
import ctypes
import os

# Load plugin library
def load_fps_plugin(plugin_lib_path):
    """
    Load FPS plugin library
    
    Args:
        plugin_lib_path: Path to libfps_plugin.so
    """
    if not os.path.exists(plugin_lib_path):
        raise FileNotFoundError(f"Plugin library not found: {plugin_lib_path}")
    
    ctypes.CDLL(plugin_lib_path)
    print(f"Loaded FPS plugin from: {plugin_lib_path}")

def register_fps_plugin(trt_logger, plugin_lib_path):
    """
    Register FPS plugin with TensorRT
    
    Args:
        trt_logger: TensorRT logger
        plugin_lib_path: Path to libfps_plugin.so
    """
    # Load the plugin library
    load_fps_plugin(plugin_lib_path)
    
    # Get plugin registry
    trt.init_libnvinfer_plugins(trt_logger, "")
    
    print("FPS plugin registered successfully")
