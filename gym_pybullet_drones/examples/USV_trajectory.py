import numpy as np
from scipy.spatial.transform import Rotation
import scipy.stats as sps
import scipy.signal as sgnl
import scipy.special as spcl
import matplotlib.pyplot as plt  # type: ignore
import matplotlib as mpl
from matplotlib import animation
from matplotlib.ticker import (MultipleLocator, AutoMinorLocator)
import math
C = 3E8


class USV_trajectory:
  def __init__(self, time, m = 1, r0 = None, v0 = None, a0 = None, φ0=None, ω0 = None, ε0 = None, anglr0 = None, total_force0 = None, gen = True):
    self.time = time
    self.m = m
    self.r = np.empty((time.n, m, 2)) # координата
    self.v = np.empty((time.n, m, 2)) # скорость
    self.a = np.empty((time.n, m, 2)) # ускорение
    self.φ = np.empty((time.n, m)) # углы Эйлера
    self.ω = np.empty((time.n, m, 1)) # угловая скорость
    self.ε = np.empty((time.n, m, 1)) # угловое ускорение
    self.anglr = np.empty((time.n, m))
    self.total_force = np.empty((time.n, m, 2))
    #self.proectia = np.empty((time.n, m, 2))
    self.r[0] = r0 if r0 is not None else np.zeros((m,2))
    self.v[0] = v0 if v0 is not None else np.zeros((m,2))
    self.a[0] = a0 if a0 is not None else np.zeros((m,2))
    self.φ[0] = φ0 if φ0 is not None else np.zeros(m)
    self.ω[0] = ω0 if ω0 is not None else np.zeros((m,1))
    self.ε[0] = ε0 if ε0 is not None else np.zeros((m,1))
    self.anglr[0] = anglr0 if anglr0 is not None else np.zeros(m)
    self.total_force[0] = total_force0 if total_force0 is not None else np.zeros((m,2))
    #self.proectia[0] = proectia0 if proectia0 is not None else np.zeros((m,2))
    if gen:
      self.ranningRandomTrajectory()

  def random_angul(self, ang, last_angul):
        angul = np.random.uniform(-ang, ang)

        return angul + last_angul

  def rotation_matrix(self, angle):
      cos_angle = np.cos(angle)
      sin_angle = np.sin(angle)
      return np.array([[cos_angle, -sin_angle], [sin_angle, cos_angle]])

  def random_force(self):
      f = np.random.uniform(1000.0, 2000.0)
      return f
  def ranningRandomTrajectory(self):

      k_a_attr = 0.1
      k_a_rep = 20
      k_a_diss = 1
      alpha = math.pi / 6
      mass = 200
      l = 2.5
      moment_inertia = mass * l*l #момент инерции
      force_engine = np.array([1,0])
      for i in range(1, self.time.n):
          for j in range(self.m):
            rs = np.delete(self.r[i-1], j, axis=0) - self.r[i-1,j]
            rs_len = np.tile(np.linalg.norm(rs, axis=1), (2,1)).T #?
            rs_dir = rs / rs_len
            a_attr = k_a_attr * np.sum(rs_dir * rs_len**2, axis=0)
            a_rep = - k_a_rep * np.sum(rs_dir/(1 + rs_len), axis=0)
            a_diss = - k_a_diss * self.v[i-1,j]
            sum_force = (a_attr + a_rep) * mass

            self.anglr[i,j] = self.random_angul(alpha, self.φ[i-1, j])
            random_traction = np.dot(self.rotation_matrix(self.anglr[i,j]),  force_engine)#cслуч тяга

            napravl = math.atan2(sum_force[1], sum_force[0])

            angle_desired = ((napravl + 2*math.pi) % (2*math.pi))
            naprel = angle_desired-((self.φ[i-1, j] + 2*math.pi) % (2*math.pi))

            if - alpha < naprel < alpha:
                proectia = np.dot(self.rotation_matrix(2*(self.φ[i-1, j]-napravl)), sum_force)
            elif alpha <= naprel <= math.pi:
                proectia = np.dot(self.rotation_matrix(self.φ[i-1, j] - alpha- napravl), sum_force)
            elif - math.pi <= naprel <= - alpha:
                proectia = np.dot(self.rotation_matrix(self.φ[i-1,j] + alpha-napravl), sum_force)


            self.total_force[i,j] = random_traction * self.random_force() + proectia

            d = np.array([l*np.cos(self.φ[i-1,j]), l*np.sin(self.φ[i-1,j]), 0])
            F = np.array([self.total_force[i,j][0], self.total_force[i,j][1], 0])
            mometn_force = np.cross(F, d)
            ε_diss = - k_a_diss *  self.ω[i-1,j]
            self.ε[i,j] = np.linalg.norm(mometn_force)*np.sign(mometn_force[2])/ moment_inertia + 2*ε_diss
            self.ω[i,j] = self.ω[i-1,j] + self.ε[i,j] * self.time.dt # углавая скорость

            orient =  self.φ[i-1,j] + self.ω[i,j] * self.time.dt
            if math.pi/2 < orient:
              self.φ[i,j] = orient - 2*math.pi
            elif orient <-math.pi/2:
              self.φ[i,j] = orient + 2*math.pi
            else:
              self.φ[i,j] = orient
            anapravl = math.atan2(a_diss[1], a_diss[0])
            deltafi = abs(anapravl-self.φ[i-1,j])
            self.a[i,j] = self.total_force[i, j] / mass + a_diss * deltafi
            self.v[i,j] = self.v[i-1,j] + self.a[i,j] * self.time.dt
            self.r[i,j] = self.r[i-1,j] + self.v[i,j] * self.time.dt



  def sample(self, fs):
      if self.time.fs%fs:
        raise ValueError(f"'fs': {fs} is not a divisor of 'self.time.fs': {self.time.fs}")
      step = self.time.fs//fs
      tr_s = USV_trajectory(time=self.time.sample(fs), m = self.m, gen = False)
      tr_s.r = self.r[::step]
      tr_s.v = self.v[::step]
      tr_s.a = self.a[::step]
      tr_s.φ = self.φ[::step]
      tr_s.ω = self.ω[::step]
      tr_s.ε = self.ε[::step]
      return tr_s
