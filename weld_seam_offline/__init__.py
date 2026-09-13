"""ROS-free port of romi-lab/robotic-welding-demo.

Load a PLY point cloud, detect the weld groove, and export a 6-DoF trajectory.
The geometry steps follow demo_all.py: asymmetry feature, DBSCAN clustering,
PCA thinning, directional sort, B-spline, and torch orientation.
"""

from .pipeline import WeldSeamResult, detect_weld_seam

__all__ = ["WeldSeamResult", "detect_weld_seam"]
__version__ = "1.0.0"
