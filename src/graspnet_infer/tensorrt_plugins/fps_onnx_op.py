"""
Custom ONNX operation for FPS
This creates a placeholder that can be replaced with TensorRT plugin
"""
import torch
import torch.nn as nn

class FPSONNXOp(nn.Module):
    """
    Custom operation that will be exported as ONNX custom op
    In TensorRT, this will be replaced with FPS plugin
    """
    def __init__(self, npoint):
        super().__init__()
        self.npoint = npoint
    
    def forward(self, xyz):
        """
        Input: xyz [B, N, 3]
        Output: idx [B, npoint] (indices)
        
        For ONNX export, we use a placeholder implementation
        that will be replaced during TensorRT build
        """
        B, N, _ = xyz.shape
        device = xyz.device
        
        # For ONNX export, we create a simple placeholder
        # This will be replaced by TensorRT plugin during engine build
        # Use a deterministic pattern that TensorRT can recognize
        indices = torch.linspace(0, N - 1, steps=self.npoint, dtype=torch.long, device=device)
        indices = indices.unsqueeze(0).expand(B, -1)
        
        return indices
