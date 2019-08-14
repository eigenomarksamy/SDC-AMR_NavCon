#! /usr/bin/python


import cubic_spline_planner


class Path:

	def __init__(self, x_path = [], y_path = []):
		self.x_path = x_path
		self.y_path = y_path

	def default_raw_path(self):
		self._x_path_file = open("PathPlanner/TextFiles/defaultPathX.txt", "r")
		self._xpath_raw = []
		for self._val in self._x_path_file.read().split():
			self._xpath_raw.append(float(self._val))
		self._x_path_file.close()
		self._y_path_file = open("PathPlanner/TextFiles/defaultPathY.txt", "r")
		self._ypath_raw = []
		for self._val in self._y_path_file.read().split():
			self._ypath_raw.append(float(self._val))
		self._y_path_file.close()
		self.x_raw = self._xpath_raw
		self.y_raw = self._ypath_raw
		return self.x_raw, self.y_raw

	def default_refactor(self):
		self._x_path = self.x_raw
		self._y_path = self.y_raw
		self._x_path = [self._counter_x_path / 2.0 for self._counter_x_path in self._x_path]
		self._y_path = [self._counter_y_path / 2.0 for self._counter_y_path in self._y_path]
		self._raw_props = self.path_props(self._x_path, self._y_path)
		self._def_x_initial = self._raw_props[0][0]
		self._def_y_initial = self._raw_props[0][1]
		self._def_x_size_path = self._raw_props[2][0]
		self._def_y_size_path = self._raw_props[2][1]
		self._def_path_size = self._def_x_size_path
		self._counter_x_path_2 = 0
		self._counter_y_path_2 = 0
		if self._def_x_initial >= 0.0:
			for self._counter_x_path_2 in range(self._def_x_size_path):
				self._x_path[self._counter_x_path_2] = self._x_path[self._counter_x_path_2] - self._def_x_initial
		else:
			for self._counter_x_path_2 in range(self._def_x_size_path):
				self._x_path[self._counter_x_path_2] = self._x_path[self._counter_x_path_2] + self._def_x_initial
		if self._def_y_initial >= 0.0:
			for self._counter_y_path_2 in range(self._def_y_size_path):
				self._y_path[self._counter_y_path_2] = self._y_path[self._counter_y_path_2] - self._def_y_initial
		else:
			for self._counter_y_path_2 in range(self._def_y_size_path):
				self._y_path[self._counter_y_path_2] = self._y_path[self._counter_y_path_2] + self._def_y_initial
		self.refacto_x = self._x_path
		self.refacto_y = self._y_path
		return self.refacto_x, self.refacto_y

	def remove_singular(self, x_path, y_path, path_props):
		self._singular_list = []
		if path_props[2][0] == path_props[2][1]:
			for i in range(path_props[2][0] - 1):
				if x_path[i] == x_path[i + 1]:
					if y_path[i] == y_path[i + 1]:
						self._singular_list.append(i)
			for j in range(len(self._singular_list)):
				x_path.pop(self._singular_list[j] - j)
				y_path.pop(self._singular_list[j] - j)
		self.x_sing = x_path
		self.y_sing = y_path
		return self.x_sing, self.y_sing

	def path_props(self, x_path, y_path):
		self._x_size_path = x_path.__len__()
		self._y_size_path = y_path.__len__()
		self._x_min_path = min(x_path)
		self._y_min_path = min(y_path)
		self._x_max_path = max(x_path)
		self._y_max_path = max(y_path)
		self._x_final_path = x_path[-1]
		self._y_final_path = y_path[-1]
		self._x_initial = x_path[0]
		self._y_initial = y_path[0]
		self._start_point = [self._x_initial, self._y_initial]
		self._goal_point = [self._x_final_path, self._y_final_path]
		self._path_len = [self._x_size_path, self._y_size_path]
		self._path_min = [self._x_min_path, self._y_min_path]
		self._path_max = [self._x_max_path, self._y_max_path]
		self.path_properties = [self._start_point, self._goal_point, self._path_len, self._path_min, self._path_max]
		return self.path_properties


def generate_path(is_default = True):
	x_path = []
	y_path = []
	if is_default == True:
		path_obj = Path()
		x_raw, y_raw = path_obj.default_raw_path()
		x_refacto, y_refacto = path_obj.default_refactor()
		path_properties = path_obj.path_props(x_refacto, y_refacto)
		x_path, y_path = path_obj.remove_singular(x_refacto, y_refacto, path_properties)
		waypoints_x = x_path
		waypoints_y = y_path
		final_goal = path_properties[1]
		course_x, course_y, course_yaw, course_k, course_s = cubic_spline_planner.calc_spline_course(waypoints_x, waypoints_y, ds=0.1)
	else:
		print "No Other than default"
	return x_path, y_path, course_x, course_y, course_yaw, course_k, course_s, final_goal
