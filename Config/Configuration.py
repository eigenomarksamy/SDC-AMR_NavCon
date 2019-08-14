#! /usr/bin/env python


class Configuration:

	def __init__(self, show_animation = True, dt = 0.1):
		self.show_animation = show_animation
		self.dt = dt


def config(show_animation, dt):
	config_obj = Configuration(show_animation, dt)
	config_list = [config_obj.show_animation, config_obj.dt]
	return config_list
