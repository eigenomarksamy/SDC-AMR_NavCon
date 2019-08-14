#! /usr/bin/env python


import sys
import math
import scipy
import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt
sys.path.append("Config/")
sys.path.append("PathPlanner/")
import Configuration
import Path


L = 1.5
max_steer = math.radians(45.0)


class State:

	def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
		self.x = x
		self.y = y
		self.yaw = yaw
		self.v = v


def update(state, a, delta, dt):
	if delta >= max_steer:
		delta = max_steer
	if delta <= - max_steer:
		delta = - max_steer
	state.x = state.x + state.v * math.cos(state.yaw) * dt
	state.y = state.y + state.v * math.sin(state.yaw) * dt
	state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
	state.v = state.v + a * dt
	return state


def pi_2_pi(angle):
	while (angle > math.pi):
		angle = angle - 2.0 * math.pi

	while (angle < -math.pi):
		angle = angle + 2.0 * math.pi
	return angle


def solve_DARE(A, B, Q, R):
	X = Q
	maxiter = 150
	eps = 0.01
	for i in range(maxiter):
		Xn = A.T * X * A - A.T * X * B * \
            	la.pinv(R + B.T * X * B) * B.T * X * A + Q
        	if (abs(Xn - X)).max() < eps:
            		X = Xn
            		break
        	X = Xn
	return Xn


def dlqr(A, B, Q, R):
	X = solve_DARE(A, B, Q, R)
	K = np.matrix(la.pinv(B.T * X * B + R) * (B.T * X * A))
	eigVals, eigVecs = la.eig(A - B * K)
	return K, X, eigVals


def lqr_steering_control(state, cx, cy, cyaw, ck, pe, pth_e, sp, dt):
	ind, e = calc_nearest_index(state, cx, cy, cyaw)
	tv = sp[ind]
	k = ck[ind]
	v = state.v
	th_e = pi_2_pi(state.yaw - cyaw[ind])
	A = np.matrix(np.zeros((5, 5)))
	A[0, 0] = 1.0
	A[0, 1] = dt
	A[1, 2] = v
	A[2, 2] = 1.0
	A[2, 3] = dt
	A[4, 4] = 1.0
	B = np.matrix(np.zeros((5, 2)))
	B[3, 0] = v / L
	B[4, 1] = dt
	Q = np.matrix(np.zeros((5, 5)))
	Q[0, 0] = 1.0
	Q[1, 1] = 1.0
	Q[2, 2] = 1.0
	Q[3, 3] = 1.0
	Q[4, 4] = 0.1
	R = np.matrix(np.zeros((2, 2)))
	R[0, 0] = 1.0
	R[1, 1] = 1.0
	K, _, _ = dlqr(A, B, Q, R)
	x = np.matrix(np.zeros((5, 1)))
	x[0, 0] = e
	x[1, 0] = (e - pe) / dt
	x[2, 0] = th_e
	x[3, 0] = (th_e - pth_e) / dt
	x[4, 0] = v - tv
	ustar = -K * x
	ff = math.atan2(L * k, 1)
	fb = pi_2_pi(ustar[0, 0])
	ai = ustar[1, 0]
	delta = ff + fb
	return delta, ind, e, th_e, ai


def calc_nearest_index(state, cx, cy, cyaw):
	dx = [state.x - icx for icx in cx]
	dy = [state.y - icy for icy in cy]
	d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
	mind = min(d)
	ind = d.index(mind)
	dxl = cx[ind] - state.x
	dyl = cy[ind] - state.y
	angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
	if angle < 0:
		mind *= -1
	return ind, mind


def closed_loop_prediction(cx, cy, cyaw, ck, speed_profile, goal, dt, show_animation):
	T = 500.0
	goal_dis = 0.3
	stop_speed = 0.05
	state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)
	time = 0.0
	x = [state.x]
	y = [state.y]
	yaw = [state.yaw]
	v = [state.v]
	t = [0.0]
	target_ind = calc_nearest_index(state, cx, cy, cyaw)
	e, e_th = 0.0, 0.0
	while T >= time:
		dl, target_ind, e, e_th, ai = lqr_steering_control(state, cx, cy, cyaw, ck, e, e_th, speed_profile, dt)
		state = update(state, ai, dl, dt)
		if abs(state.v) <= stop_speed:
			target_ind += 1
		time = time + dt
		dx = state.x - goal[0]
		dy = state.y - goal[1]
		if math.sqrt(dx ** 2 + dy ** 2) <= goal_dis:
			print("Goal")
			break
		x.append(state.x)
		y.append(state.y)
		yaw.append(state.yaw)
		v.append(state.v)
		t.append(time)
		if target_ind % 1 == 0 and show_animation:
			plot_loop(cx, cy, x, y, target_ind, state)
	return t, x, y, yaw, v


def calc_speed_profile(cx, cy, cyaw, target_speed):
	speed_profile = [target_speed] * len(cx)
	direction = 1.0
	for i in range(len(cx) - 1):
		dyaw = abs(cyaw[i + 1] - cyaw[i])
		switch = math.pi / 4.0 <= dyaw < math.pi / 2.0
		if switch:
			direction *= -1
		if direction != 1.0:
			speed_profile[i] = - target_speed
		else:
			speed_profile[i] = target_speed
		if switch:
			speed_profile[i] = 0.0
	for i in range(40):
		speed_profile[-i] = target_speed / (50 - i)
		if speed_profile[-i] <= 1.0 / 3.6:
			speed_profile[-i] = 1.0 / 3.6
	return speed_profile


def get_path():
	waypoints_x = []
	waypoints_y = []
	waypoints_x, waypoints_y, course_x, course_y, course_yaw, course_k, course_s, final_goal = Path.generate_path(is_default = True)
	target_velocity = 10.0 / 3.6
	return waypoints_x, waypoints_y, final_goal, course_x, course_y, course_yaw, course_k, course_s, target_velocity


def plot_loop(cx, cy, x, y, target_ind, state):
	plt.cla()
	plt.plot(cx, cy, "-r", label="course")
	plt.plot(x, y, "ob", label="trajectory")
	plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
	plt.axis("equal")
	plt.grid(True)
	plt.title("speed[km/h]:" + str(round(state.v * 3.6, 2)) +",target index:" + str(target_ind))
	plt.pause(0.0001)


def plot_main(ax, ay, cx, cy, cyaw, ck, s, x, y):
	plt.close()
	flg, _ = plt.subplots(1)
	plt.plot(ax, ay, "xb", label="waypoints")
	plt.plot(cx, cy, "-r", label="target course")
	plt.plot(x, y, "-g", label="tracking")
	plt.grid(True)
	plt.axis("equal")
	plt.xlabel("x[m]")
	plt.ylabel("y[m]")
	plt.legend()
	flg, ax = plt.subplots(1)
	plt.plot(s, [math.degrees(iyaw) for iyaw in cyaw], "-r", label="yaw")
	plt.grid(True)
	plt.legend()
	plt.xlabel("line length[m]")
	plt.ylabel("yaw angle[deg]")
	flg, ax = plt.subplots(1)
	plt.plot(s, ck, "-r", label="curvature")
	plt.grid(True)
	plt.legend()
	plt.xlabel("line length[m]")
	plt.ylabel("curvature [1/m]")
	plt.show()


def main():
	print("LQR steering control tracking start!!")
	configurations = Configuration.config(True, 0.1)
	show_animation = configurations[0]
	dt = configurations[1]
	ax, ay, goal, cx, cy, cyaw, ck, s, target_speed = get_path()
	sp = calc_speed_profile(cx, cy, cyaw, target_speed)
	t, x, y, yaw, v = closed_loop_prediction(cx, cy, cyaw, ck, sp, goal, dt, show_animation)
	if show_animation:
		plot_main(ax, ay, cx, cy, cyaw, ck, s, x, y)


if __name__ == '__main__':
	main()
