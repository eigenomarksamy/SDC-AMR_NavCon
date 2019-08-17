#! /usr/bin/env python


class Configuration:

	def get_config(self):
		self._conf_file = open("Config/TextFiles/ModelConfigurationParameters.txt", "r")
		self._config_text = []
		for self._text in self._conf_file.read().split():
			self._config_text.append(self._text)
		self._conf_file.close()


	def refactor(self):
		self.num_list = []
		self.par_list = []
		self.inc_par = 0
		self.inc_num = 1
		while self.inc_par < len(self._config_text):
			self.par_list.append(self._config_text[self.inc_par])
			self.inc_par += 2
		while self.inc_num < len(self._config_text):
			self.num_list.append(float(self._config_text[self.inc_num]))
			self.inc_num += 2
		return self.par_list, self.num_list


	def par_to_val(self, parameter):
		index = self.par_list.index(parameter)
		return self.num_list[index]


	def configure(self):
		self.get_config()
		self.refactor()
		self.L = self.par_to_val('wheel_base')
		self.B = self.par_to_val('track_width')
		self.max_steer = self.par_to_val('max_steer')
		self.show_animation = self.par_to_val('show_animation')
		if self.show_animation > 0:
			self.show_animation = True
		else:
			self.show_animation = False
		self.dt = self.par_to_val('dt')
		return self.L, self.B, self.max_steer, self.show_animation, self.dt


def config():
	config_obj = Configuration()
	L, B, max_steer, show_animation, dt = config_obj.configure()
	config_list = [show_animation, dt, L, B, max_steer]
	return config_list
