"""Educational 3D, seven-revolute-joint (7-DOF) arm and sphere collision model.

Joint-space angles are radians throughout the implementation. The arm uses a
specified sequence of local joint axes and subsequent local-z link offsets;
this is an illustrative serial chain, NOT the calibrated model of a real robot.
"""
from dataclasses import dataclass, field
import numpy as np

AXES = ('z', 'y', 'x', 'y', 'x', 'y', 'x')
LINK_LENGTHS = np.array([.34, .30, .28, .25, .23, .18, .12])  # metres
JOINT_LIMITS_DEG = np.array([
    [-170., 170.], [-120., 120.], [-170., 170.], [-120., 120.],
    [-170., 170.], [-120., 120.], [-175., 175.],
])


def rot_batch(axis: str, a: np.ndarray) -> np.ndarray:
    """Vectorized active rotation about a joint's *local* axis."""
    c, s = np.cos(a), np.sin(a)
    R = np.zeros((len(a), 3, 3), dtype=float)
    if axis == 'x':
        R[:, 0, 0] = 1
        R[:, 1, 1], R[:, 1, 2] = c, -s
        R[:, 2, 1], R[:, 2, 2] = s, c
    elif axis == 'y':
        R[:, 1, 1] = 1
        R[:, 0, 0], R[:, 0, 2] = c, s
        R[:, 2, 0], R[:, 2, 2] = -s, c
    elif axis == 'z':
        R[:, 2, 2] = 1
        R[:, 0, 0], R[:, 0, 1] = c, -s
        R[:, 1, 0], R[:, 1, 1] = s, c
    else:
        raise ValueError(f'unknown axis {axis}')
    return R




def segment_distance_batch(p1, q1, p2, q2):
    """Shortest distance between two 3D line segments, for (M,3) arrays.

    Vectorized adaptation of the standard closest-points-on-segments equations.
    """
    u = q1 - p1
    v = q2 - p2
    w = p1 - p2
    aa = np.einsum('ij,ij->i', u, u)
    bb = np.einsum('ij,ij->i', u, v)
    cc = np.einsum('ij,ij->i', v, v)
    dd = np.einsum('ij,ij->i', u, w)
    ee = np.einsum('ij,ij->i', v, w)
    D = aa * cc - bb * bb
    eps = 1e-12
    sN = bb * ee - cc * dd
    tN = aa * ee - bb * dd
    sD = D.copy()
    tD = D.copy()
    parallel = D < eps
    sN = np.where(parallel, 0., sN)
    sD = np.where(parallel, 1., sD)
    tN = np.where(parallel, ee, tN)
    tD = np.where(parallel, cc, tD)
    low = (~parallel) & (sN < 0.)
    sN = np.where(low, 0., sN)
    tN = np.where(low, ee, tN)
    tD = np.where(low, cc, tD)
    high = (~parallel) & (sN > sD)
    sN = np.where(high, sD, sN)
    tN = np.where(high, ee + bb, tN)
    tD = np.where(high, cc, tD)

    low_t = tN < 0.
    tN = np.where(low_t, 0., tN)
    sN = np.where(low_t & (-dd < 0.), 0., sN)
    sN = np.where(low_t & (-dd > aa), sD, sN)
    middle = low_t & (-dd >= 0.) & (-dd <= aa)
    sN = np.where(middle, -dd, sN)
    sD = np.where(middle, aa, sD)

    high_t = tN > tD
    tN = np.where(high_t, tD, tN)
    expr = -dd + bb
    sN = np.where(high_t & (expr < 0.), 0., sN)
    sN = np.where(high_t & (expr > aa), sD, sN)
    middle = high_t & (expr >= 0.) & (expr <= aa)
    sN = np.where(middle, expr, sN)
    sD = np.where(middle, aa, sD)

    sc = np.divide(sN, sD, out=np.zeros_like(sN), where=np.abs(sD) > eps)
    tc = np.divide(tN, tD, out=np.zeros_like(tN), where=np.abs(tD) > eps)
    delta = w + sc[:, None] * u - tc[:, None] * v
    return np.linalg.norm(delta, axis=1)

@dataclass
class Robot7DOF:
    lengths: np.ndarray = field(default_factory=lambda: LINK_LENGTHS.copy())
    limits: np.ndarray = field(default_factory=lambda: np.deg2rad(JOINT_LIMITS_DEG))
    # x,y,z,r in metres. Scene constants can be overridden by users/tests.
    obstacles: np.ndarray = field(default_factory=lambda: np.array([
        [.10, .20, 1.16, .12],
        [-.29, .20, .85, .11],
    ], dtype=float))
    link_radius: float = .026  # capsule thickness approximation
    safety_margin: float = .024
    self_collision: bool = True  # nonadjacent capsule pairs checked by default

    def fk(self, q: np.ndarray) -> np.ndarray:
        return self.fk_batch(np.asarray(q, dtype=float)[None, :])[0]

    def fk_batch(self, q: np.ndarray) -> np.ndarray:
        q = np.atleast_2d(np.asarray(q, dtype=float))
        if q.shape[1] != 7:
            raise ValueError('Expected shape (7,) or (M,7)')
        m = len(q)
        pts = np.zeros((m, 8, 3), dtype=float)
        R = np.broadcast_to(np.eye(3), (m, 3, 3)).copy()
        for i, (axis, length) in enumerate(zip(AXES, self.lengths)):
            R = R @ rot_batch(axis, q[:, i])
            pts[:, i + 1] = pts[:, i] + length * R[:, :, 2]
        return pts

    def obstacle_clearance(self, q: np.ndarray, points=None) -> np.ndarray:
        """Min distance from arm capsule surface to obstacle sphere surfaces (m).

        Positive = separated; zero = touching; negative = intersecting.
        Includes every physical link, not just the end-effector.
        """
        pts = self.fk_batch(q) if points is None else points
        if not len(self.obstacles):
            return np.full(len(pts), np.inf)
        a, b = pts[:, :-1, :], pts[:, 1:, :]
        ab = b - a
        centers, radii = self.obstacles[:, :3], self.obstacles[:, 3]
        ac = centers[None, None, :, :] - a[:, :, None, :]
        proj = np.sum(ac * ab[:, :, None, :], axis=-1)
        segnorm = np.sum(ab ** 2, axis=-1)[:, :, None]
        t = np.clip(proj / np.maximum(segnorm, 1e-12), 0., 1.)
        nearest = a[:, :, None, :] + t[..., None] * ab[:, :, None, :]
        dist = np.linalg.norm(nearest - centers[None, None, :, :], axis=-1)
        return (dist - radii[None, None, :] - self.link_radius).min(axis=(1, 2))

    def self_clearance(self, q: np.ndarray, points=None) -> np.ndarray:
        """Min surface gap between non-adjacent arm links (capsule model)."""
        pts = self.fk_batch(q) if points is None else points
        pairs = [(i, j) for i in range(7) for j in range(i + 2, 7)]
        left = np.array([i for i, _ in pairs], dtype=int)
        right = np.array([j for _, j in pairs], dtype=int)
        m = len(pts)
        p1, q1 = pts[:, left], pts[:, left + 1]
        p2, q2 = pts[:, right], pts[:, right + 1]
        gap = segment_distance_batch(p1.reshape(-1, 3), q1.reshape(-1, 3),
                                     p2.reshape(-1, 3), q2.reshape(-1, 3))
        return gap.reshape(m, -1).min(axis=1) - 2 * self.link_radius

    def valid(self, q: np.ndarray) -> np.ndarray:
        q = np.atleast_2d(np.asarray(q, dtype=float))
        in_limits = np.all((q >= self.limits[:, 0] - 1e-10) &
                           (q <= self.limits[:, 1] + 1e-10), axis=1)
        points = self.fk_batch(q)
        collision_free = self.obstacle_clearance(q, points) > self.safety_margin
        if self.self_collision and np.any(in_limits & collision_free):
            eligible = in_limits & collision_free
            self_ok = self.self_clearance(q[eligible], points[eligible]) > self.safety_margin
            collision_free[eligible] &= self_ok
        return in_limits & collision_free

    def edge_free(self, a: np.ndarray, b: np.ndarray, step: float = .055) -> bool:
        """Check interpolated full-arm configurations, not merely endpoints.

        `step` caps each individual joint's interpolation change (radians).
        Continuous-time collision freedom is not mathematically guaranteed by
        discrete sampling; decrease `step` for more conservative checking.
        """
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(2, int(np.ceil(np.max(np.abs(b - a)) / step)) + 1)
        qq = a[None, :] + np.linspace(0, 1, n)[:, None] * (b - a)[None, :]
        return bool(np.all(self.valid(qq)))

    def sample(self, rng: np.random.Generator) -> np.ndarray:
        return rng.uniform(self.limits[:, 0], self.limits[:, 1])
