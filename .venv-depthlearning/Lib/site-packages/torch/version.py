from typing import Optional

__all__ = ['__version__', 'debug', 'cuda', 'git_version', 'hip', 'rocm', 'xpu']
__version__ = '2.14.1+cpu'
debug = False
cuda: Optional[str] = None
git_version = '5c4886908584029761b579af026dcfb627c84070'
hip: Optional[str] = None
rocm: Optional[str] = None
xpu: Optional[str] = None
