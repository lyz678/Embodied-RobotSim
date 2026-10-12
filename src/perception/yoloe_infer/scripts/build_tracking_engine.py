#!/usr/bin/env python3
"""Expose a YOLOE neck tensor in an existing ONNX graph and build TensorRT.

The default tensor is P3, the feature consumed by model.22 in the bundled YOLOE
v8l graph. For another architecture pass its actual pre-head feature tensor name.
No model weights are changed and no second ReID network is introduced.
"""
import argparse
from pathlib import Path
import subprocess
import onnx
from onnx import helper, TensorProto


def expose_neck(source, target, tensor):
    model = onnx.load(str(source))
    names = {name for node in model.graph.node for name in node.output}
    if tensor not in names:
        raise ValueError(f"Neck tensor does not exist: {tensor}; inspect your ONNX head inputs")
    if any(o.name == 'reid_p3' for o in model.graph.output):
        raise ValueError('Graph already exposes reid_p3')
    model.graph.node.append(helper.make_node('Cast', [tensor], ['reid_p3'],
                                            name='tracking_neck_fp32', to=TensorProto.FLOAT))
    # Let ONNX infer the true NCHW feature dimensions; never assume the detection
    # or mask-prototype output is a neck feature.
    model.graph.output.append(helper.make_tensor_value_info('reid_p3', TensorProto.FLOAT, [None]*4))
    model = onnx.shape_inference.infer_shapes(model)
    shape = next(o for o in model.graph.output if o.name == 'reid_p3').type.tensor_type.shape
    if len(shape.dim) != 4 or any(d.dim_value <= 0 for d in shape.dim):
        raise ValueError('Neck output requires a fully static NCHW shape')
    onnx.checker.check_model(model)
    onnx.save(model, str(target))
    print('Neck shape:', [d.dim_value for d in shape.dim], flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True, help='New .onnx path; engine uses the same stem')
    p.add_argument('--tensor', default='/model.15/cv2/act/Mul_output_0')
    p.add_argument('--trtexec', default='/usr/src/tensorrt/bin/trtexec')
    p.add_argument('--onnx-only', action='store_true')
    a = p.parse_args()
    if a.source.resolve() == a.output.resolve() or a.output.exists() or a.output.with_suffix('.engine').exists():
        p.error('Choose new output paths to preserve existing model artifacts')
    a.output.parent.mkdir(parents=True, exist_ok=True)
    expose_neck(a.source, a.output, a.tensor)
    if not a.onnx_only:
        help_text = subprocess.run([a.trtexec, '--help'], capture_output=True, text=True, check=True).stdout
        # TRT 11 is strongly typed and removed the builder --fp16 switch.
        precision = ['--fp16'] if '--fp16' in help_text else []
        subprocess.run([a.trtexec, f'--onnx={a.output}', *precision, '--skipInference',
                        f'--saveEngine={a.output.with_suffix(".engine")}'], check=True)


if __name__ == '__main__':
    main()
