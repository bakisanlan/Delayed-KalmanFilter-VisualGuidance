"""
Pinhole + Radtan distortion camera model.

Implements the OpenCV-convention radial-tangential distortion model
on top of a standard pinhole camera. Provides projection, FOV check,
and Gaussian pixel noise injection.
"""

import numpy as np
from typing import Optional, Tuple


class PinholeRadtanCamera:
    """Pinhole camera with radial-tangential (radtan) distortion."""

    def __init__(
        self,
        K: np.ndarray,
        dist_coeffs: np.ndarray,
        image_width: int,
        image_height: int,
        R_c2b: np.ndarray,
        noise_sigma: float = 1.0,
    ):
        """
        Parameters
        ----------
        K : np.ndarray, shape (3, 3)
            Camera intrinsic matrix::

                [[fx,  0, cx],
                 [ 0, fy, cy],
                 [ 0,  0,  1]]

        dist_coeffs : np.ndarray, shape (4,)
            Radial-tangential distortion coefficients [k1, k2, p1, p2].

        image_width : int
            Image width in pixels.

        image_height : int
            Image height in pixels.

        R_c2b : np.ndarray, shape (3, 3)
            Rotation matrix from camera frame to body frame.

        noise_sigma : float
            Standard deviation of additive Gaussian pixel noise (pixels).
        """
        self.K = np.array(K, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(dist_coeffs, dtype=np.float64).ravel()
        assert self.dist_coeffs.shape[0] == 4, "Expected 4 distortion coefficients [k1, k2, p1, p2]"

        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.R_c2b = np.array(R_c2b, dtype=np.float64).reshape(3, 3)
        self.R_b2c = self.R_c2b.T  # body-to-camera (inverse)
        self.noise_sigma = float(noise_sigma)

        # Extract intrinsics for convenience
        self.fx = self.K[0, 0]
        self.fy = self.K[1, 1]
        self.cx = self.K[0, 2]
        self.cy = self.K[1, 2]

    # --------------------------------------------------------------------- #
    # Projection
    # --------------------------------------------------------------------- #

    def project(self, p_cam: np.ndarray) -> Optional[Tuple[float, float]]:
        """
        Project a 3-D point (in camera frame) onto the image plane with
        radial-tangential distortion.

        Parameters
        ----------
        p_cam : np.ndarray, shape (3,)
            Point in the camera coordinate frame (x-right, y-down, z-forward).

        Returns
        -------
        (u, v) : tuple of float, or None
            Pixel coordinates.  ``None`` if the point is behind the camera
            (z <= 0).
        """
        x, y, z = p_cam.ravel()
        if z <= 0.0:
            return None  # behind camera

        # Normalised image coordinates
        xn = x / z
        yn = y / z

        # Distortion ----------------------------------------------------------
        k1, k2, p1, p2 = self.dist_coeffs
        r2 = xn * xn + yn * yn
        r4 = r2 * r2

        # Radial
        radial = 1.0 + k1 * r2 + k2 * r4

        # Tangential
        xd = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
        yd = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn

        # Pixel coordinates
        u = self.fx * xd + self.cx
        v = self.fy * yd + self.cy

        return (u, v)

    # --------------------------------------------------------------------- #
    # FOV check
    # --------------------------------------------------------------------- #

    def is_in_fov(self, u: float, v: float) -> bool:
        """Return True if pixel (u, v) lies within the image boundaries."""
        return 0.0 <= u < self.image_width and 0.0 <= v < self.image_height

    # --------------------------------------------------------------------- #
    # Noise
    # --------------------------------------------------------------------- #

    def add_noise(self, u: float, v: float) -> Tuple[float, float]:
        """Add zero-mean Gaussian noise to pixel coordinates."""
        u_noisy = u + np.random.normal(0.0, self.noise_sigma)
        v_noisy = v + np.random.normal(0.0, self.noise_sigma)
        return (u_noisy, v_noisy)
