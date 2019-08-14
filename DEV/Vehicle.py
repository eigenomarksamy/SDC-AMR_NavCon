#! /usr/bin/env python


import math


class Vehicle:

	def __init__(self):
		self.vparam_maxsteer = math.radians(45.0)
		self.vparam_m = 1140.0
		self.vparam_iz = 1436.24
		self.vparam_lf = 1.165
		self.vparam_lr = self.vparam_lf
		self.vparam_lt = self.vparam_lf + self.vparam_lr
		self.vparam_cf = 155494.663
		self.vparam_cr = self.vparam_cf

	def dynamic_model(self, is_initialization = True, vx):
		self.vparam_vx = vx
		if vparam_vx == 0.0:
			vparam_vx = 0.001
		if is_initialization:
			self._a11 = 0.0
			self._a12 = 1.0
			self._a13 = 0.0
			self._a14 = 0.0
			self._a21 = 0.0
			self._a22 = (-(self.vparam_cf + self.vparam_cr)/(self.vparam_m * self.vparam_vx))
			self._a23 = ((self.vparam_cr + self.vparam_cf)/self.vparam_m)
			self._a24 = ((self.vparam_lr * self.vparam_cr - self.vparam_lf * self.vparam_cf)/(self.vparam_m * self.vparam_vx))
			self._a31 = 0.0
			self._a32 = 0.0
			self._a33 = 0.0
			self._a34 = 1.0
			self._a41 = 0.0
			self._a42 = ((self.vparam_lr * self.vparam_cr - self.vparam_lf * self.vparam_cf)/(self.vparam_iz * self.vparam_vx))
			self._a43 = ((self.vparam_lf * self.vparam_cf - self.vparam_lr * self.vparam_cr)/self.vparam_iz)
			self._a44 = (-((self.vparam_lf**2)*self.vparam_cf + (self.vparam_lr**2)*self.vparam_cr)/(self.vparam_iz * self.vparam_vx))
			self._b11 = 0.0
			self._b21 = (self.vparam_cf/self.vparam_m)
			self._b31 = 0.0
			self._b41 = ((self.vparam_lf * self.vparam_cf) / self.vparam_m)
			self._c11 = 0.0
			self._d11 = 0.0
		else:
			self._a22 = (-(self.vparam_cf + self.vparam_cr)/(self.vparam_m * self.vparam_vx))
			self._a24 = ((self.vparam_lr * self.vparam_cr - self.vparam_lf * self.vparam_cf)/(self.vparam_m * self.vparam_vx))
			self._a42 = ((self.vparam_lr * self.vparam_cr - self.vparam_lf * self.vparam_cf)/(self.vparam_iz * self.vparam_vx))
			self._a44 = (-((self.vparam_lf**2)*self.vparam_cf + (self.vparam_lr**2)*self.vparam_cr)/(self.vparam_iz * self.vparam_vx))

	def matrix_form(self):
		self.A = np.matrix([
							[self._a11, self._a12, self._a13, self._a14], 
							[self._a21, self._a22, self._a23, self._a24], 
							[self._a31, self._a32, self._a33, self._a34], 
							[self._a41, self._a42, self._a43, self._a44]
							])
		self.B = np.matrix([
							[self._b11], 
							[self._b21], 
							[self._b31], 
							[self._b41]
							])
		self.C = np.matrix([
							[self._c11]
							])
		self.D = np.matrix([
							[self._d11]
							])

	def discretize(self):
		self._ts = self.config_params.dt
		self._I = np.identity(self.A.shape[0] & self.A.shape[1])
		self.Ad = np.zeros((self.A.shape[0] & self.A.shape[1]))
		self.Bd = np.zeros((self.B.shape[0] & self.B.shape[1]))
		self.Ad = self._I + self.A * self._ts
		self.Bd = self.B * self._ts
		return self.Ad, self. Bd

	def update_dynamic_model(self, is_initialization = False, vx):
		self.dynamic_model(is_initialization, vx)
		self.matrix_form()
		A, B = self.discretize()
		return A, B

	def update_state(self, A, B):
		

	def get_vehicle_configurations(self, is_kinematic = True):
		model_parameters_list = []
		if is_kinematic:
			return [self.vparam_maxsteer, 
					self.vparam_lt]
		else:
			return [self.vparam_maxsteer, 
					self.vparam_lf, self.vparam_lr, self.vparam_lt, 
					self.vparam_m, self.vparam_iz, 
					self.vparam_cf, self.vparam_cr]