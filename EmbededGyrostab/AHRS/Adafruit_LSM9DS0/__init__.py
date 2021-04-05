try:
    from .LSM9DS0 import *
except Exception: #ImportError
    from LSM9DS0 import *