"""Replacement 800x800 fisheye camera specification (not calibrated).

Projection: provisional equidistant fisheye, r = f * theta.
The stated 3.15 mm physical focal length is retained as metadata; without the
sensor's active image dimensions it cannot independently determine pixel focal
length. Pixel focal length below is derived from the stated 160-degree diagonal
FOV and the 800x800 image diagonal.
"""
import math
import numpy as np

IMAGE_WIDTH_PX = 800
IMAGE_HEIGHT_PX = 800
DIAGONAL_FOV_DEG = 160.0
PHYSICAL_FOCAL_LENGTH_MM = 3.15
PROJECTION_MODEL = "equidistant_fisheye"

CX = (IMAGE_WIDTH_PX - 1) / 2.0
CY = (IMAGE_HEIGHT_PX - 1) / 2.0
FISHEYE_F_PX = math.hypot(CX, CY) / math.radians(DIAGONAL_FOV_DEG / 2.0)
FX = FISHEYE_F_PX
FY = FISHEYE_F_PX
CAMERA_MATRIX = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])

# Unknown because this lens/camera cannot be calibrated. Do not use the old
# OV5647 6 mm pinhole distortion coefficients for the replacement fisheye.
DIST_COEFFS = None
CALIBRATED = False
WARNING = "Provisional planning-only 160deg diagonal equidistant fisheye model."
