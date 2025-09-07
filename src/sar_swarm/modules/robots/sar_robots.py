# Implement basic SAR robot class extending IR-SIM robot
# Add battery management (simple linear drain)
# Create robot state tracking

# class SARRobot for with attributes: robot_id, battery_level, detected_survivors, explored_cells, and current_task, with functions to update_battery and get sensor_data

class SARRobot:
    def __init__(self, robot_id, **kwargs):
        self.robot_id = robot_id
        self.battery_level = 100  # Battery level percentage
        self.detected_survivors = []  # List to store detected survivors
        self.explored_cells = set()  # Set to store explored cells
        self.current_task = "exploration"  # Current task assigned to the robot

    def update_battery(self, amount):
        """Update the battery level by a certain amount."""
        self.battery_level = max(0, min(100, self.battery_level + amount))

    def get_sensor_data(self):
        """Simulate getting sensor data."""
        # This is a placeholder for actual sensor data retrieval logic
        # Code to Get LiDAR data and camera data
        # Simulate LiDAR data as a list of distances (in meters)
        lidar_data = [1.2, 2.5, 0.8, 3.0, 2.2]  # Example values

        # Simulate camera data as a dictionary with detected objects
        camera_data = {
            "objects": [
            {"type": "survivor", "position": (5, 10)},
            {"type": "obstacle", "position": (3, 7)},
            ]
        }
    
        return {
            "battery_level": self.battery_level,
            "detected_survivors": self.detected_survivors,
            "explored_cells": list(self.explored_cells),
            "current_task": self.current_task,
        }