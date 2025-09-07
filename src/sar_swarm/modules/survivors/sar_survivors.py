# Create 5 survivor objects with different properties
# Implement heat signatures and visibility
# Place survivors randomly behind obstacles
# Class SARSurvivor for survivors with attributes: survivor_id, position, heat_signature, visibility, detected by [robot_ids]
class SARSurvivor:
    def __init__(self, survivor_id, position, heat_signature=1.0, visibility=True):
        self.survivor_id = survivor_id
        self.position = position  # (x, y) coordinates
        self.heat_signature = heat_signature  # Heat signature intensity
        self.visibility = visibility  # Whether the survivor is visible to cameras
        self.detected_by = []  # List of robot_ids that have detected this survivor

    def mark_detected(self, robot_id):
        """Mark this survivor as detected by a specific robot."""
        if robot_id not in self.detected_by:
            self.detected_by.append(robot_id)
    # Function to calculate detection probability based on distance
    def detection_probability(self, robot_position):
        """Calculate detection probability based on distance and visibility."""
        if not self.visibility:
            return 0.0
        distance = ((self.position[0] - robot_position[0]) ** 2 + (self.position[1] - robot_position[1]) ** 2) ** 0.5
        # Simple model: probability decreases with distance
        prob = max(0.0, 1.0 - (distance / 20.0))  # Assuming max effective range is 20 meters
        return prob * self.heat_signature  # Modulate by heat signature
    