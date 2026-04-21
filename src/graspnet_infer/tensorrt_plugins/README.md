# TensorRT FPS Plugin

这个目录包含用于在 TensorRT 中实现真正的 FPS (Furthest Point Sampling) 的自定义插件。

## 文件说明

- `fps_plugin.cu`: CUDA 内核实现 FPS 算法
- `fps_plugin.h`: TensorRT 插件头文件
- `fps_plugin.cpp`: TensorRT 插件实现
- `CMakeLists.txt`: CMake 构建配置
- `register_plugin.py`: Python 插件注册工具
- `fps_onnx_op.py`: ONNX 操作占位符（可选）

## 编译插件

```bash
cd tensorrt_plugins
mkdir build && cd build
cmake ..
make
```

编译完成后，会生成 `libfps_plugin.so` 文件。

## 使用方法

### 1. 导出 ONNX 模型

```bash
python export_onnx.py --checkpoint_path <checkpoint> --output_path graspnet.onnx
```

注意：`export_onnx.py` 现在使用真正的 FPS 实现（从 pointnet2._ext），以保证精度。

### 2. 构建 TensorRT Engine

```bash
python build_engine.py --onnx_path graspnet.onnx --engine_path graspnet.trt --plugin_lib tensorrt_plugins/build/libfps_plugin.so
```

如果插件库在默认位置，可以省略 `--plugin_lib` 参数。

## 工作原理

1. **ONNX 导出**: 使用真正的 FPS CUDA 实现导出 ONNX 模型
2. **TensorRT 构建**: 
   - 注册 FPS 插件
   - 解析 ONNX 模型
   - 如果遇到 FPS 相关操作，使用插件替换（需要手动实现节点替换逻辑）

## 注意事项

- TensorRT 的 ONNX parser 可能不认识 FPS 操作
- 如果 ONNX 解析失败，可能需要：
  1. 使用 ONNX graph optimization 工具预处理模型
  2. 手动替换 ONNX 图中的 FPS 节点
  3. 或者使用 polygraphy 等工具进行转换

## 精度保证

插件使用与原始 CUDA 实现相同的算法，确保精度一致：
- 迭代最远点采样
- 相同的距离计算
- 相同的选择策略
