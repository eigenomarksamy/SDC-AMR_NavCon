#! /usr/bin/env python

import numpy as np

class Plant:

	def __init__(self, state_plane, input_plane, output_plane, dt = 0.1):
		self.dt = dt
		self.A = np.matrix(np.zeros((state_plane, state_plane)))
		self.B = np.matrix(np.zeros((state_plane, input_plane)))
		self.C = np.matrix(np.zeros((output_plane, state_plane)))
		self.D = np.matrix(np.zeros((output_plane, input_plane)))
		self.x = np.matrix(np.zeros((state_plane, 1)))


	def init_state(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
		state = State(x, y, yaw, v)
		return state
	

	def input_update(self, state, a, delta, dt, max_steer, L):
		if delta >= max_steer:
			delta = max_steer
		if delta <= - max_steer:
			delta = - max_steer
		state.x = state.x + state.v * np.cos(state.yaw) * dt
		state.y = state.y + state.v * np.sin(state.yaw) * dt
		state.yaw = state.yaw + state.v / L * np.tan(delta) * dt
		state.v = state.v + a * dt
		return state


	def model_update(self, v, L):
		self.A[0, 0] = 1.0
		self.A[0, 1] = self.dt
		self.A[1, 2] = v
		self.A[2, 2] = 1.0
		self.A[2, 3] = self.dt
		self.A[4, 4] = 1.0
		self.B[3, 0] = v / L
		self.B[4, 1] = self.dt
		return self.A, self.B


	def state_update(self, e, pe, dt, th_e, pth_e, v, tv):
		self.x[0, 0] = e
		self.x[1, 0] = (e - pe) / dt
		self.x[2, 0] = th_e
		self.x[3, 0] = (th_e - pth_e) / dt
		self.x[4, 0] = v - tv
		return self.x


class State:

	def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
		self.x = x
		self.y = y
		self.yaw = yaw
		self.v = v


def set_plant(state_plane, input_plane, output_plane, dt = 0.1):
	plant_obj = Plant(state_plane, input_plane, output_plane, dt = 0.1)
	return plant_obj
