#!/usr/bin/env python3

"""
planner.py
donkeycar part for controlling the car.

@authors: Jason Mayeda, Sidney Hsu, Roy Sun, Matthew Gilli
"""


class Mixer():
    def run(self, steering_person, steering_planner):
        print("person = " + str(steering_person) + ", planner = " + str(steering_planner))
        if abs(steering_person) < 0.05:
            if steering_planner > 1:
                return 0.5
            if steering_planner < -1:
                return -0.5
            return steering_planner/2
        print("person " + str(steering_person))
        return max(-1, min(1, steering_person + 0.2 * steering_planner))

    def shutdown(self):
        return
