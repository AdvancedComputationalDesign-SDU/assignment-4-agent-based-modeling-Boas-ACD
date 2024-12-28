"""
Helper Functions for Assignment 4

This module contains helper components which assist in the agent-based model.
They should be viewed as an component that would be put into the main Simulation Visualization:

from src.helper_functions import my_helper_function
"""
"""
Main agent function: a shopper which starts at an entrance and goes to a target
"""
import Rhino
import rhinoscriptsyntax as rs
import Rhino.Geometry as rg

class Agent:
    def __init__(self, srf, start_position):
        self.position = rg.Point3d(*start_position) # to ensure correct datatype
        self.velocity = rg.Vector3d(0, 0, 0) # to ensure correct datatype
        self.srf = srf

    def dist(self, target_point):
        """
        Calculate distance between between agent & target
        """
        target = rg.Point3d(*target_point) # to ensure correct datatype
        return self.position.DistanceTo(target)

    def move(self):
        "Move & update the agent's position"
        # Update position
        self.position += self.velocity

    def seek_target(self, target, max_speed, slow_radius):
        """
        Adjust velocity to seek target
        """
        target_point = rg.Point3d(*target)
        target_vector = target_point - self.position  # Vector pointing to the target
        distance = target_vector.Length  # Distance to the target

        if distance > 0:
            target_vector.Unitize()  # Normalize
            desired_velocity = target_vector * max_speed
            if distance < slow_radius:
                # Slow down as it approaches the target
                desired_velocity *= distance / slow_radius
            self.velocity = desired_velocity
    """
    Output: Agent
    """
