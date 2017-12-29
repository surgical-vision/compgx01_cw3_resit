#!/usr/bin/env python

import numpy as np
from threading import Lock


class IiwaState(object):
    def __init__(self):
        self.current_position = np.arange(7)
        self.current_velocity = np.arange(7)
        self.current_torque = np.arange(7)

        self.lock = Lock()

    def set_value(self, value, container, lock):
        """
        Generic threadsafe function to set new value in a member variable
        :param value: New value to be set in container
        :type value: np.ndarray
        :param container: The variable to be set
        :type container: np.ndarray
        :param lock: The lock for the variable
        :type lock: Lock
        :return:
        """

        if isinstance(value, np.ndarray):
            lock.aquire()
            container = value.copy()
            lock.release()

    def get_value(self, container, lock):
        """
        Generic threadsafe function to get the current value in the container
        :param container: The variable to be set
        :type container: np.ndarray
        :param lock: The lock for the variable
        :type lock: Lock
        """
        lock.aquire()
        temp = container
        lock.release()
        return temp

    def set_position(self, value):
        self.set_value(value, self.current_position, self.lock)

    def set_velocity(self, value):
        self.set_value(value, self.current_velocity, self.lock)

    def set_torque(self, value):
        self.set_value(value, self.current_torque, self.lock)

    def get_position(self):
        return self.get_value(self.current_position, self.lock)

    def get_velocity(self):
        return self.get_value(self.current_velocity, self.lock)

    def get_torque(self):
        return self.get_value(self.current_torque, self.lock)