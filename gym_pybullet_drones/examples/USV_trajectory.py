import numpy as np
import matplotlib.pyplot as plt  # type: ignore
import math

C = 3E8
K_A_ATTR = 0.05
K_A_REP = 20
K_A_DISS = 5
ALPHA = math.pi / 6
MASS = 200
L = 2.5
MOMENT_INERTIA = MASS * L ** 2
FORCE_ENGINE = np.array([1, 0])
MAX_THRUST = 5000


class UsvTrajectory:

    def __init__(self,
                 time,
                 m=1,
                 r0=None,
                 v0=None,
                 a0=None,
                 φ0=None,
                 ω0=None,
                 ε0=None,
                 angle0=None,
                 total_traction0=None,
                 xyz0=None,
                 gen=True):
        self.time = time
        self.m = m
        self.xyz = np.empty((time.n, m, 3))
        self.r = np.empty((time.n, m, 2))
        self.v = np.empty((time.n, m, 2))
        self.a = np.empty((time.n, m, 2))
        self.φ = np.empty((time.n, m))
        self.ω = np.empty((time.n, m, 1))
        self.ε = np.empty((time.n, m, 1))
        self.angle = np.empty((time.n, m))
        self.total_traction = np.empty((time.n, m, 2))
        self.r[0] = r0 if r0 is not None else np.zeros((m, 2))
        self.v[0] = v0 if v0 is not None else np.zeros((m, 2))
        self.a[0] = a0 if a0 is not None else np.zeros((m, 2))
        self.φ[0] = φ0 if φ0 is not None else np.zeros(m)
        self.ω[0] = ω0 if ω0 is not None else np.zeros((m, 1))
        self.ε[0] = ε0 if ε0 is not None else np.zeros((m, 1))
        self.angle[0] = angle0 if angle0 is not None else np.zeros(m)
        self.total_traction[0] = total_traction0 if total_traction0 is not None else np.zeros((m, 2))
        self.xyz[0] = xyz0 if xyz0 is not None else np.zeros((m, 3))

        if gen:
            self.running_random_trajectory()

    @staticmethod
    def random_angle(alpha, last_angle):
        angle = np.random.uniform(-alpha, alpha)
        return angle + last_angle

    @staticmethod
    def rotation_matrix(angle):
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        return np.array([[cos_angle, -sin_angle], [sin_angle, cos_angle]])

    @staticmethod
    def random_force():
        f = np.random.uniform(1000.0, 2000.0)
        return f

    def running_random_trajectory(self):

        for i in range(1, self.time.n):
            for j in range(self.m):
                rs = np.delete(self.r[i - 1], j, axis=0) - self.r[i - 1, j]
                rs_len = np.tile(np.linalg.norm(rs, axis=1), (2, 1)).T
                rs_dir = rs / rs_len
                a_attr = K_A_ATTR * np.sum(rs_dir * rs_len ** 2, axis=0)
                a_rep = - K_A_REP * np.sum(rs_dir / (1 + rs_len), axis=0)
                a_diss = - K_A_DISS * self.v[i - 1, j]
                sum_force = (a_attr + a_rep) * MASS

                self.angle[i, j] = self.random_angle(ALPHA, self.φ[i - 1, j])
                random_traction = np.dot(self.rotation_matrix(self.angle[i, j]), FORCE_ENGINE)
                direction = math.atan2(sum_force[1], sum_force[0])
                angle_desired = ((direction + math.pi) % (2 * math.pi) - math.pi)
                direction_sum_force = angle_desired - ((self.φ[i - 1, j] + math.pi) % (2 * math.pi) - math.pi)

                if - ALPHA < direction_sum_force < ALPHA:
                    traction_desired = np.dot(self.rotation_matrix(2 * (self.φ[i - 1, j] - direction)), sum_force)
                elif ALPHA <= direction_sum_force <= math.pi:
                    traction_desired = np.dot(self.rotation_matrix(self.φ[i - 1, j] - ALPHA - direction), sum_force)
                elif - math.pi <= direction_sum_force <= - ALPHA:
                    traction_desired = np.dot(self.rotation_matrix(self.φ[i - 1, j] + ALPHA - direction), sum_force)

                self.total_traction[i, j] = random_traction * self.random_force() + traction_desired

                if np.linalg.norm(self.total_traction[i, j]) > MAX_THRUST:
                    traction = (self.total_traction[i, j] / np.linalg.norm(self.total_traction[i, j])) * MAX_THRUST
                else:
                    traction = self.total_traction[i, j]
                arm = np.array([L * np.cos(self.φ[i - 1, j]), L * np.sin(self.φ[i - 1, j]), 0])
                force = np.array([traction[0], traction[1], 0])
                moment_force = np.cross(force, arm)
                ε_diss = - K_A_DISS * self.ω[i - 1, j]
                self.ε[i, j] = np.linalg.norm(moment_force) * np.sign(moment_force[2]) / MOMENT_INERTIA + 2 * ε_diss
                self.ω[i, j] = self.ω[i - 1, j] + self.ε[i, j] * self.time.dt

                orient = self.φ[i - 1, j] + self.ω[i, j] * self.time.dt
                if math.pi < orient:
                    self.φ[i, j] = orient - 2 * math.pi
                elif orient < -math.pi:
                    self.φ[i, j] = orient + 2 * math.pi
                else:
                    self.φ[i, j] = orient
                direction_a_diss = math.atan2(a_diss[1], a_diss[0])
                deltafi = abs(direction_a_diss - self.φ[i - 1, j])
                self.a[i, j] = traction / MASS + a_diss * deltafi
                self.v[i, j] = self.v[i - 1, j] + self.a[i, j] * self.time.dt
                self.r[i, j] = self.r[i - 1, j] + self.v[i, j] * self.time.dt
                self.xyz[i, j] = np.array([self.r[i, j, 0], self.r[i, j, 1], 0])

    def sample(self, fs):
        if self.time.fs % fs:
            raise ValueError(f"'fs': {fs} is not a divisor of 'self.time.fs': {self.time.fs}")
        step = self.time.fs // fs
        tr_s = UsvTrajectory(time=self.time.sample(fs), m=self.m, gen=False)
        tr_s.r = self.r[::step]
        tr_s.v = self.v[::step]
        tr_s.a = self.a[::step]
        tr_s.φ = self.φ[::step]
        tr_s.ω = self.ω[::step]
        tr_s.ε = self.ε[::step]
        tr_s.xyz = self.xyz[::step]
        return tr_s
