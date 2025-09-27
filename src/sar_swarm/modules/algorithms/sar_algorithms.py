# Implement simple sensor fusion (voting system)
# Create detection confidence scoring
# Add false positive filtering

# Class SensorFusion for fusing data from multiple sensors with attributes: fused_detections, and methods to fuse_data, filter_false_positives, detection confidence scoring, simple voting system for detection

class SensorFusion:
    def __init__(self):
        self.fused_detections = []  # List to store fused detections

    def fuse_data(self, sensor_data_list):
        """Fuse data from multiple sensors."""
        detection_map = {}
        for sensor_data in sensor_data_list:
            for detection in sensor_data:
                key = (detection['type'], detection['position'])
                if key not in detection_map:
                    detection_map[key] = {'count': 0, 'confidence': 0.0}
                detection_map[key]['count'] += 1
                detection_map[key]['confidence'] += detection.get('confidence', 1.0)

        # Create fused detections with average confidence
        self.fused_detections = [
            {
                'type': key[0],
                'position': key[1],
                'confidence': value['confidence'] / value['count'],
                'votes': value['count']
            }
            for key, value in detection_map.items()
        ]

    # Function process_detections with parameters: rgb_detections, thermal_detections, robot_position
    def process_detections(self, rgb_detections, thermal_detections, robot_position):
        """Process and fuse detections from RGB and thermal cameras."""
        high_confidence_detection = "High"
        low_confidence_detection = "Low"
        # Add confidence based on detection of rgb_detections and thermal_detections
       
        if rgb_detections and thermal_detections:
            return high_confidence_detection
        elif rgb_detections or thermal_detections:
            return low_confidence_detection

    def filter_false_positives(self, threshold=0.5):
        """Filter out detections with confidence below a certain threshold."""
        self.fused_detections = [
            detection for detection in self.fused_detections
            if detection['confidence'] >= threshold
        ]


# Implement basic occupancy grid mapping
# Add explored area tracking
# Create simple localization
# Class SimpleMapping for mapping and localization with attributes: update_occupancy_grid, explored_area, robot_position, lidar_data, methods to update_map, mark_explored_cells_in_area, localize_robot
class SimpleMapping:
    def __init__(self, grid_size=(100, 100), cell_size=1):
        self.grid_size = grid_size  # Size of the occupancy grid (width, height)
        self.cell_size = cell_size  # Size of each cell in meters
        self.occupancy_grid = [[0 for _ in range(grid_size[1])] for _ in range(grid_size[0])]  # 0: free, 1: occupied
        self.explored_area = set()  # Set to store explored cells
        self.robot_position = (0, 0)  # Robot's current position (x, y)

    def update_map(self, robot_position, lidar_data):
        """Update the occupancy grid based on LiDAR data."""
        self.robot_position = robot_position
        for angle, distance in lidar_data:
            if distance < 0 or distance > 30:  # Ignore invalid distances
                continue
            x_end = robot_position[0] + distance * cos(angle)
            y_end = robot_position[1] + distance * sin(angle)
            self._mark_occupied_cells(robot_position, (x_end, y_end))

    def _mark_occupied_cells(self, start, end):
        """Mark cells as occupied along the line from start to end."""
        x0, y0 = int(start[0] / self.cell_size), int(start[1] / self.cell_size)
        x1, y1 = int(end[0] / self.cell_size), int(end[1] / self.cell_size)
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if 0 <= x0 < self.grid_size[0] and 0 <= y0 < self.grid_size[1]:
                self.occupancy_grid[x0][y0] = 1  # Mark as occupied
                self.explored_area.add((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            err2 = err * 2
            if err2 > -dy:
                err -= dy
                x0 += sx
            if err2 < dx:
                err += dx
                y0 += sy            
    def mark_explored_cells_in_area(self, area):
        """Mark cells in a given area as explored."""
        for cell in area:
            x, y = int(cell[0] / self.cell_size), int(cell[1] / self.cell_size)
            if 0 <= x < self.grid_size[0] and 0 <= y < self.grid_size[1]:
                self.explored_area.add((x, y))      
    def localize_robot(self, sensor_data):
        """Simple localization based on sensor data (placeholder)."""
        # In a real scenario, implement a localization algorithm (e.g., particle filter)
        # Here we just return the current robot position
        return self.robot_position
from math import cos, sin
# Implement frontier detection algorithm
# Create frontier prioritization (closest first)
# Add frontier assignment to robots
# Class FrontierFinder for detecting and managing frontiers with attributes: detect_frontiers, prioritize_frontiers, assign_frontier_to_robot
class FrontierFinder:
    def __init__(self, occupancy_grid, cell_size=1):
        self.occupancy_grid = occupancy_grid  # 2D list representing the occupancy grid
        self.cell_size = cell_size  # Size of each cell in meters
        self.frontiers = []  # List to store detected frontiers

    def detect_frontiers(self):
        """Detect frontiers in the occupancy grid."""
        self.frontiers = []
        rows = len(self.occupancy_grid)
        cols = len(self.occupancy_grid[0]) if rows > 0 else 0

        for x in range(1, rows - 1):
            for y in range(1, cols - 1):
                if self.occupancy_grid[x][y] == 0:  # Free cell
                    # Check neighboring cells for unknown (unexplored) cells
                    neighbors = [
                        self.occupancy_grid[x-1][y], self.occupancy_grid[x+1][y],
                        self.occupancy_grid[x][y-1], self.occupancy_grid[x][y+1]
                    ]
                    if any(n == -1 for n in neighbors):  # -1 represents unknown
                        self.frontiers.append((x * self.cell_size, y * self.cell_size))

    def prioritize_frontiers(self, robot_position):
        """Prioritize frontiers based on distance from the robot."""
        def distance(frontier):
            return ((frontier[0] - robot_position[0]) ** 2 + (frontier[1] - robot_position[1]) ** 2) ** 0.5

        self.frontiers.sort(key=distance)

    def assign_frontier_to_robot(self, robot_id, robot_position):
        """Assign the closest frontier to a robot."""
        if not self.frontiers:
            return None
        closest_frontier = min(self.frontiers, key=lambda f: ((f[0] - robot_position[0]) ** 2 + (f[1] - robot_position[1]) ** 2) ** 0.5)
        self.frontiers.remove(closest_frontier)
        return closest_frontier     

# Implement A* pathfinding algorithm
# Add basic collision avoidance
# Create path following behaviour

# Class SimplePathPlanner for planning paths with attributes: plan_path, avoid_obstacles, avoid_robots, smooth_path
class SimplePathPlanner:
    def __init__(self, occupancy_grid, cell_size=1):
        self.occupancy_grid = occupancy_grid  # 2D list representing the occupancy grid
        self.cell_size = cell_size  # Size of each cell in meters

    def plan_path(self, start, goal):
        """Plan a path from start to goal using A* algorithm."""
        from queue import PriorityQueue

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        start_cell = (int(start[0] / self.cell_size), int(start[1] / self.cell_size))
        goal_cell = (int(goal[0] / self.cell_size), int(goal[1] / self.cell_size))

        open_set = PriorityQueue()
        open_set.put((0, start_cell))
        came_from = {}
        g_score = {start_cell: 0}
        f_score = {start_cell: heuristic(start_cell, goal_cell)}

        while not open_set.empty():
            current = open_set.get()[1]

            if current == goal_cell:
                return self._reconstruct_path(came_from, current)

            neighbors = self._get_neighbors(current)
            for neighbor in neighbors:
                tentative_g_score = g_score[current] + 1  # Assume cost between cells is 1
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_cell)
                    if neighbor not in [i[1] for i in open_set.queue]:
                        open_set.put((f_score[neighbor], neighbor))

        return []  # No path found

    def _get_neighbors(self, cell):
        """Get walkable neighboring cells."""
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        neighbors = []
        for d in directions:
            neighbor = (cell[0] + d[0], cell[1] + d[1])
            if (0 <= neighbor[0] < len(self.occupancy_grid) and
                0 <= neighbor[1] < len(self.occupancy_grid[0]) and
                self.occupancy_grid[neighbor[0]][neighbor[1]] == 0):    # Free cell         
                neighbors.append(neighbor)
        return neighbors
    def _reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal."""
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        total_path.reverse()
        return [(cell[0] * self.cell_size, cell[1] * self.cell_size) for cell in total_path]
    def smooth_path(self, path):
        """Smooth the path to avoid sharp turns (placeholder)."""
        # In a real scenario, implement a path smoothing algorithm
        return path
    def avoid_obstacles(self, path):
        """Modify path to avoid obstacles (placeholder)."""
        # In a real scenario, implement obstacle avoidance logic
        return path
    def avoid_robots(self, path, other_robot_positions):
        """Modify path to avoid other robots (placeholder)."""
        # In a real scenario, implement robot avoidance logic
        return path

# Implement basic separation behavior
# Create search formation coordination
# Add dynamic spacing control
# Class SwarmBehavior for managing swarm behaviors with attributes: maintain_spacing, coordinate_search, 
class SwarmBehavior:
    def __init__(self, desired_spacing=2.0):
        self.desired_spacing = desired_spacing  # Desired spacing between robots in meters

    def maintain_spacing(self, robot_position, other_robot_positions):
        """Adjust position to maintain desired spacing from other robots."""
        adjustment = (0.0, 0.0)
        for other_pos in other_robot_positions:
            distance = ((robot_position[0] - other_pos[0]) ** 2 + (robot_position[1] - other_pos[1]) ** 2) ** 0.5
            if distance < self.desired_spacing and distance > 0:
                # Move away from the other robot
                adjustment = (
                    adjustment[0] + (robot_position[0] - other_pos[0]) / distance,
                    adjustment[1] + (robot_position[1] - other_pos[1]) / distance
                )
        return (robot_position[0] + adjustment[0], robot_position[1] + adjustment[1])

    def coordinate_search(self, robot_id, robot_position, frontiers):
        """Assign a frontier to the robot for searching."""
        if not frontiers:
            return None
        # Simple assignment: assign the closest frontier
        closest_frontier = min(frontiers, key=lambda f: ((f[0] - robot_position[0]) ** 2 + (f[1] - robot_position[1]) ** 2) ** 0.5)
        return closest_frontier

    def dynamic_spacing_control(self, current_spacing):
        """Adjust desired spacing based on current conditions (placeholder)."""
        # In a real scenario, implement logic to adjust spacing dynamically
        return self.desired_spacing
    
# Implement greedy task allocation
# Create task prioritization
# Add task reassignment system
# Class TaskAllocator for allocating tasks with attributes: assign_tasks, prioritize_tasks, reassign_tasks
class TaskAllocator:
    def __init__(self):
        self.tasks = []  # List to store tasks

    def assign_tasks(self, robots, tasks):
        """Assign tasks to robots based on proximity and capability."""
        assignments = {}
        for robot in robots:
            if not tasks:
                break
            closest_task = min(tasks, key=lambda t: ((t['position'][0] - robot['position'][0]) ** 2 + (t['position'][1] - robot['position'][1]) ** 2) ** 0.5)
            assignments[robot['id']] = closest_task
            tasks.remove(closest_task)
        return assignments

    def prioritize_tasks(self, tasks):
        """Prioritize tasks based on urgency and importance."""
        self.tasks = sorted(tasks, key=lambda t: (t['urgency'], t['importance']), reverse=True)

    def reassign_on_failure(self, robot_id, failed_task, assignments):
        """Reassign a task if a robot fails to complete it."""
        if robot_id in assignments and assignments[robot_id] == failed_task:
            del assignments[robot_id]
            return True
        return False
    
# Implement voting system for detections
# Create group decision making
# Add response coordination
# Class SwarmDecisionMaking for managing group decisions with attributes: vote_on_decision, coordinate_response
class SwarmDecisionMaking:
    def __init__(self):
        self.votes = {}  # Dictionary to store votes for decisions

    def vote_on_decision(self, robot_id, decision):
        """Record a vote from a robot on a specific decision."""
        if decision not in self.votes:
            self.votes[decision] = set()
        self.votes[decision].add(robot_id)

    def get_decision(self):
        """Determine the decision with the most votes."""
        if not self.votes:
            return None
        return max(self.votes.items(), key=lambda item: len(item[1]))[0]

    def coordinate_response(self, decision):
        """Coordinate a response based on the group decision (placeholder)."""
        # In a real scenario, implement logic to coordinate actions based on the decision
        return f"Coordinating response for decision: {decision}"
    
# Implement 2-robot confirmation requirement
# Create confirmation timeout system
# Add survivor validation logic
# Class ConfirmationSystem for managing confirmations with attributes: request_confirmation, validate_survivor
class ConfirmationSystem:
    def __init__(self, confirmation_timeout=10):
        self.confirmation_timeout = confirmation_timeout  # Time in seconds to wait for confirmation
        self.confirmations = {}  # Dictionary to store confirmations

    def request_confirmation(self, robot_id, detection):
        """Request confirmation from another robot for a detection."""
        if detection not in self.confirmations:
            self.confirmations[detection] = set()
        self.confirmations[detection].add(robot_id)

    def validate_survivor(self, detection):
        """Validate a survivor detection based on confirmations."""
        if detection in self.confirmations and len(self.confirmations[detection]) >= 2:
            return True
        return False

# Add environmental factor consideration
# Implement basic detection learning
# Improve false positive reduction
# Class ImprovedDetection for advanced detection with attributes: consider_environment, learn_from_detections
class ImprovedDetection:
    def __init__(self):
        self.detection_history = []  # List to store past detections

    def consider_environment(self, detection, environment_factors):
        """Adjust detection confidence based on environmental factors."""
        adjusted_confidence = detection['confidence']
        for factor, impact in environment_factors.items():
            if factor in detection:
                adjusted_confidence *= impact
        detection['confidence'] = adjusted_confidence
        return detection

    def learn_from_detections(self, new_detection):
        """Learn from past detections to improve future accuracy."""
        self.detection_history.append(new_detection)
        # Simple learning: increase confidence for repeated detections of the same type and position
        similar_detections = [d for d in self.detection_history if d['type'] == new_detection['type'] and d['position'] == new_detection['position']]
        if len(similar_detections) > 1:
            new_detection['confidence'] += 0.1 * (len(similar_detections) - 1)  # Increase confidence
            new_detection['confidence'] = min(new_detection['confidence'], 1.0)  # Cap at 1.0
        return new_detection
    def reduce_false_positives(self, detections, threshold=0.5):
        """Reduce false positives by filtering detections below a confidence threshold."""
        return [d for d in detections if d['confidence'] >= threshold]
    
    
# Implement survivor detection tracking
# Create location estimation from multiple robots
# Add detection confidence updates
# Class SurvivorTracker for tracking survivors with attributes: track_detections, estimate_location, update_confidence
class SurvivorTracker:
    def __init__(self):
        self.tracked_detections = {}  # Dictionary to store tracked detections

    def track_detections(self, robot_id, detection):
        """Track detections from multiple robots."""
        key = (detection['type'], detection['position'])
        if key not in self.tracked_detections:
            self.tracked_detections[key] = {'robots': set(), 'confidence': 0.0}
        self.tracked_detections[key]['robots'].add(robot_id)
        self.tracked_detections[key]['confidence'] += detection.get('confidence', 1.0)

    def estimate_location(self, detection):
        """Estimate the location of a survivor based on multiple detections."""
        key = (detection['type'], detection['position'])
        if key in self.tracked_detections:
            robots = self.tracked_detections[key]['robots']
            avg_position = detection['position']  # In a real scenario, average positions from multiple detections
            return avg_position, len(robots)
        return detection['position'], 1

    def update_confidence(self, detection):
        """Update the confidence of a tracked detection."""
        key = (detection['type'], detection['position'])
        if key in self.tracked_detections:
            self.tracked_detections[key]['confidence'] += detection.get('confidence', 1.0)
            self.tracked_detections[key]['confidence'] = min(self.tracked_detections[key]['confidence'], 1.0)  # Cap at 1.0
            return self.tracked_detections[key]['confidence']
        return detection.get('confidence', 1.0)
    

