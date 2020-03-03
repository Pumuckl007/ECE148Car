#!/usr/bin/env python3

"""
planner.py
donkeycar part for controlling the car.

@authors: Jason Mayeda, Sidney Hsu, Roy Sun, Matthew Gilli
"""


class Mixer():
    def run(self, steering_person, steering_planner):
        if abs(steering_person) < 0.05:
            return steering_planner
        return steering_person

    def shutdown(self):
        return
