# Create RGB camera simulator (distance + line-of-sight)
# Create thermal camera simulator (heat signature detection)
# Integrate sensors with IR-SIM's existing LiDAR

# Class RBGCamera for RGB camera sensor with attributes: Simple distance-based detection 

class RBGCamera:
    def __init__(self, range_meters=10):
        self.range_meters = range_meters  # Detection range in meters

    def detect_objects(self, environment, robot_position):
        """Simulate object detection within the camera's range."""
        detected_objects = []
        for obj in environment.get_objects():
            distance = ((obj['position'][0] - robot_position[0]) ** 2 + (obj['position'][1] - robot_position[1]) ** 2) ** 0.5
            if distance <= self.range_meters:
                detected_objects.append(obj)
        return detected_objects
    
# Class ThermalCamera for thermal camera sensor with attributes: Simple heat signature detection
class ThermalCamera:
    def __init__(self, range_meters=15):
        self.range_meters = range_meters  # Detection range in meters

    def detect_heat_signatures(self, environment, robot_position):
        """Simulate heat signature detection within the camera's range."""
        detected_signatures = []
        for obj in environment.get_objects():
            if obj['type'] == 'survivor':
                distance = ((obj['position'][0] - robot_position[0]) ** 2 + (obj['position'][1] - robot_position[1]) ** 2) ** 0.5
                if distance <= self.range_meters:
                    detected_signatures.append(obj)
        return detected_signatures    