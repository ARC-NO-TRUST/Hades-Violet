__version__ = '0.22.0'
git_version = '9eb57cd5c96be7fe31923eb65399c3819d064587'
from torchvision.extension import _check_cuda_version
if _check_cuda_version() > 0:
    cuda = _check_cuda_version()
