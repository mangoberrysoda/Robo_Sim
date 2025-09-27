from math import cos, pi, sin
from typing import Optional

import matplotlib.transforms as mtransforms
import numpy as np
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from shapely import MultiLineString, Point, is_valid, prepare
from shapely.ops import unary_union

from irsim.config import env_param
from irsim.util.util import (
    WrapTo2Pi,
    geometry_transform,
    transform_point_with_state,
)


class RGBCamTest:
    """
    Simulates a RGB camera sendor detecting a survivor

    Args:
        state (np.ndarray): Initial state of the sensor.
        obj_id (int): ID of the associated object.
        range_min (float): Minimum detection range.
        range_max (float): Maximum detection range.
        FOV (float): Field of view of the camera in radians.
        noise (bool): Whether noise is added to measurements.
        std (float): Standard deviation for range noise.
        angle_std (float): Standard deviation for angle noise.
        offset (list): Offset of the sensor from the object's position.
        alpha (float): Transparency for plotting.
        has_velocity (bool): Whether the sensor measures velocity.
        **kwargs: Additional arguments.
            color (str): Color of the sensor.

    Let's reuse these params

    Attr:
        - sensor_type (str): Type of sensor ("lidar2d"). Default is "lidar2d".
        - range_min (float): Minimum detection range in meters. Default is 0.
        - range_max (float): Maximum detection range in meters. Default is 10.
        - FOV (float): Total angle range of the sensor in radians. Default is pi. WrapTo2Pi is applied.
        - angle_min (float): Starting angle of the sensor's scan relative to the forward direction in radians. Calculated as -FOV / 2.
        - angle_max (float): Ending angle of the sensor's scan relative to the forward direction in radians. Calculated as FOV / 2.
        - angle_inc (float): Angular increment between each laser beam in radians. Calculated as FOV / number.
        - number (int): Number of laser beams. Default is 100.
        - scan_time (float): Time taken to complete one full scan in seconds. Default is 0.1.
        - noise (bool): Whether to add noise to the measurements. Default is False.
        - std (float): Standard deviation for range noise in meters. Effective only if `noise` is True. Default is 0.2.
        - angle_std (float): Standard deviation for angle noise in radians. Effective only if `noise` is True. Default is 0.02.
        - offset (np.ndarray): Offset of the sensor relative to the object's position, formatted as [x, y, theta]. Default is [0, 0, 0].
        - lidar_origin (np.ndarray): Origin position of the Lidar sensor, considering offset and the object's state.
        - alpha (float): Transparency level for plotting the laser beams. Default is 0.3.
        - has_velocity (bool): Whether the sensor measures the velocity of detected points. Default is False.
        - velocity (np.ndarray): Velocity data for each laser beam, formatted as (2, number) array. Effective only if `has_velocity` is True. Initialized to zeros.
        - time_inc (float): Time increment for each scan, simulating the sensor's time resolution. Default is 5e-4.
        - range_data (np.ndarray): Array storing range data for each laser beam. Initialized to `range_max` for all beams.
        - angle_list (np.ndarray): Array of angles corresponding to each laser beam, distributed linearly from `angle_min` to `angle_max`.
        - color (str): Color of the sensor's representation in visualizations. Default is "r" (red).
        - obj_id (int): ID of the associated object, used to differentiate between multiple sensors or objects in the environment. Default is 0.
        - plot_patch_list (list): List storing plot patches (e.g., line collections) for visualization purposes.
        - plot_line_list (list): List storing plot lines for visualization purposes.
        - plot_text_list (list): List storing plot text elements for visualization purposes.
    """

    def __init__(
        self,
        state: Optional[np.ndarray] = None,
        obj_id: int = 0,
        range_min: float = 0.3, #shortest distance camera can see
        range_max: float = 3, #distance camera can see 
        FOV: float = 1.22173,  #radians (converted from 70 degrees)
        noise: bool = False,   # default no noise
        std: float = 0.2,
        angle_std: float = 0.02,
        offset: Optional[list[float]] = None,
        alpha: float = 0.3,
        has_velocity: bool = False,
        **kwargs,
    ) -> None:
        """
        Initialize the RGBCamera sensor.


        """
        if offset is None:
            offset = [0, 0, 0]
        self.sensor_type = "camera_rgb"

        self.range_min = range_min
        self.range_max = range_max

        self.FOV = WrapTo2Pi(FOV)
        self.angle_min = -self.FOV / 2
        self.angle_max = self.FOV / 2
        self.number = 100
        self.angle_list = np.linspace(self.angle_min, self.angle_max, num=self.number)

        self.noise = noise
        self.std = std
        self.angle_std = angle_std
        self.offset = np.c_[offset]


        self.alpha = alpha
        self.has_velocity = has_velocity

        self.time_inc = 0.05 # camera frame rate 20Hz
        self.human_detection = None  # Detected human Details

        self._state = state
        self.init_geometry(self._state)
        self.count = 0   # count the number of detections

        self.color = kwargs.get("color", "b")   # blue for camera

        self.obj_id = obj_id

        # these are for plotting, we will come back to this later after functional testing
        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

    def init_geometry(self, state):
        """
        Initialize the Camera's searching geometry.

        Args:
            state (np.ndarray): Current state of the sensor.
        """
        segment_point_list = []

        for i in range(self.number):
            x = self.range_max * cos(self.angle_list[i])
            y = self.range_max * sin(self.angle_list[i])

            point0 = np.zeros((1, 2))
            point = np.array([[x], [y]]).T

            segment = np.concatenate((point0, point), axis=0)

            segment_point_list.append(segment)

        self.origin_state = self.offset
        geometry = MultiLineString(segment_point_list)
        self._original_geometry = geometry_transform(geometry, self.origin_state)
        self.camera_origin = transform_point_with_state(self.offset, state)

        self._geometry = geometry_transform(self._original_geometry, state)
        self._init_geometry = self._geometry

    def step(self, state):
        """
        Update the Camera's state and process detection with environment objects.

        Args:
            state (np.ndarray): New state of the sensor.
        """
        self._state = state

        self.camera_origin = transform_point_with_state(self.offset, self._state)
        new_geometry = geometry_transform(self._original_geometry, self._state)
        prepare(new_geometry)

        new_geometry, intersect_indices = self.camera_geometry_process(new_geometry)

        
        self._geometry = new_geometry
        self.populate_detection_data(intersect_indices)


    def camera_geometry_process(self, camera_geometry):
        """
        Find the intersected objects and return the intersected indices with the camera geometry

        Args:
            camera_geometry (shapely.geometry.MultiLineString): The geometry of the camera.

        Returns:
            list: The indices of the intersected objects.
        """

        object_tree = env_param.GeometryTree
        objects = env_param.objects
        geometries = [obj._geometry for obj in objects]

        # Guard against missing geometry index
        if object_tree is None:
            return camera_geometry, []

        potential_geometries_index = object_tree.query(camera_geometry)

        geometries_to_subtract = []
        intersect_indices = []

        for geom_index in potential_geometries_index:
            geo = geometries[geom_index]
            obj = objects[geom_index]

            if (
                obj._id == self.obj_id
                or not is_valid(obj._geometry)
                or obj.unobstructed
            ):
                continue

            if obj.shape == "human":

                if camera_geometry.intersects(geo):
                    geometries_to_subtract.append(geo)
                    intersect_indices.append(geom_index)

        if geometries_to_subtract:
            merged_geometry = unary_union(geometries_to_subtract)
            camera_geometry = camera_geometry.difference(merged_geometry)

        return camera_geometry, intersect_indices

    def populate_detection_data(self, intersect_indices):
        """
        Polulate the detection data based in the current geometry.
        """
        if len(intersect_indices) == 0:
            self.det_x_distance = None
            self.det_y_distance = None
            self.det_theta = None
            self.det_id = None
            return
        else:
        # self.det_id = str(self.obj_id) + str(self.count)
        # self.count += 1

            for index, line in enumerate(self._geometry.geoms, intersect_indices ):
                
                # self.det_x_distance = line.length + np.random.normal(0, self.std)
                # self.det_y_distance = 0 + np.random.normal(0, self.std)
                # self.det_theta = self.angle_list[index] + np.random.normal(0, self.angle_std)
                
                if line.length < self.range_max - 0.02:
                    for index_obj in intersect_indices:
                        obj = env_param.objects[index_obj]
                        if obj.geometry.distance(line) < 0.1:



    def get_human_detections(self):
        """
        GetRGB human detection data.

        Returns:
            dict: detection data including id, location
        """
        human_data = {}
        # we need to calculate these values in this class and keep
        human_data["id"] = self.det_id
        human_data["x"] = self.det_x_distance
        human_data["y"] = self.det_y_distance
        human_data["theta"] = self.det_theta
        # so now if any function calls this function outside it will get where is the human with respect to the robot(camera)
        return human_data

    def get_offset(self):
        """
        Get the sensor's offset.

        Returns:
            list: Offset as a list.
        """
        return np.squeeze(self.offset).tolist()

    def plot(self, ax, state: Optional[np.ndarray] = None, **kwargs):
        """
        Plot the Lidar's detected lines on a given axis.
        """
        if state is None:
            state = self.state

        self._plot(ax, state, **kwargs)

    def _init_plot(self, ax, **kwargs):
        """
        Initialize the plot for the Lidar.
        """
        self._plot(ax, self.origin_state, **kwargs)

    @property
    def state(self) -> np.ndarray:
        """
        Get the current state of the lidar sensor.

        Returns:
            np.ndarray: Current state of the sensor.
        """
        return self._state

    def _plot(self, ax, state, **kwargs):
        """
        Plot the Lidar's detected lines using the specified state for positioning.
        Creates line segments in local coordinates and applies transforms to position them.

        Args:
            ax: Matplotlib axis.
            state: State vector [x, y, theta, ...] defining lidar position and orientation.
            **kwargs: Plotting options.
        """
        lines = []

        if isinstance(ax, Axes3D):
            # For 3D plotting, calculate actual world coordinates since transforms don't work the same way
            if state is not None and len(state) > 0:
                # Calculate lidar position based on object state and sensor offset
                lidar_x = self.lidar_origin[0, 0]
                lidar_y = self.lidar_origin[1, 0]
                lidar_theta = (
                    self.lidar_origin[2, 0] if self.lidar_origin.shape[0] > 2 else 0
                )
            else:
                lidar_x, lidar_y, lidar_theta = 0, 0, 0

            # Create line segments in world coordinates for 3D
            for i in range(self.number):
                x_local = self.range_data[i] * cos(self.angle_list[i])
                y_local = self.range_data[i] * sin(self.angle_list[i])

                # Transform to world coordinates
                x_world = (
                    lidar_x + x_local * cos(lidar_theta) - y_local * sin(lidar_theta)
                )
                y_world = (
                    lidar_y + x_local * sin(lidar_theta) + y_local * cos(lidar_theta)
                )

                start_point = np.array([lidar_x, lidar_y, 0])
                end_point = np.array([x_world, y_world, 0])
                segment = [start_point, end_point]
                lines.append(segment)

            self.laser_LineCollection = Line3DCollection(
                lines, linewidths=1, colors=self.color, alpha=self.alpha, zorder=2
            )
            ax.add_collection3d(self.laser_LineCollection)
        else:
            # For 2D plotting, create line segments in local coordinates and use transforms
            for i in range(self.number):
                x = self.range_data[i] * cos(self.angle_list[i])
                y = self.range_data[i] * sin(self.angle_list[i])
                segment = [np.array([0, 0]), np.array([x, y])]
                lines.append(segment)

            self.laser_LineCollection = LineCollection(
                lines, linewidths=1, colors=self.color, alpha=self.alpha, zorder=2
            )
            ax.add_collection(self.laser_LineCollection)

            # Apply transform for 2D case - use provided state for positioning
            if state is not None and len(state) > 0:
                lidar_x = self.lidar_origin[0, 0]
                lidar_y = self.lidar_origin[1, 0]
                lidar_theta = (
                    self.lidar_origin[2, 0] if self.lidar_origin.shape[0] > 2 else 0
                )

                # Create transform: rotate by lidar orientation, then translate to lidar position
                trans = (
                    mtransforms.Affine2D()
                    .rotate(lidar_theta)
                    .translate(lidar_x, lidar_y)
                    + ax.transData
                )
                self.laser_LineCollection.set_transform(trans)

        self.plot_patch_list.append(self.laser_LineCollection)

    def _step_plot(self):
        """
        Update the lidar visualization using matplotlib transforms based on current state.
        Creates line segments in local coordinates and applies transform to position them.
        """
        if not hasattr(self, "laser_LineCollection"):
            return

        ax = self.laser_LineCollection.axes
        lines = []

        if isinstance(ax, Axes3D):
            # For 3D plotting, calculate actual world coordinates
            lidar_x = self.lidar_origin[0, 0]
            lidar_y = self.lidar_origin[1, 0]
            lidar_theta = (
                self.lidar_origin[2, 0] if self.lidar_origin.shape[0] > 2 else 0
            )

            # Create line segments in world coordinates for 3D
            for i in range(self.number):
                x_local = self.range_data[i] * cos(self.angle_list[i])
                y_local = self.range_data[i] * sin(self.angle_list[i])

                # Transform to world coordinates
                x_world = (
                    lidar_x + x_local * cos(lidar_theta) - y_local * sin(lidar_theta)
                )
                y_world = (
                    lidar_y + x_local * sin(lidar_theta) + y_local * cos(lidar_theta)
                )

                start_point = np.array([lidar_x, lidar_y, 0])
                end_point = np.array([x_world, y_world, 0])
                segment = [start_point, end_point]
                lines.append(segment)
        else:
            # For 2D plotting, create line segments in local coordinates
            for i in range(self.number):
                x = self.range_data[i] * cos(self.angle_list[i])
                y = self.range_data[i] * sin(self.angle_list[i])
                segment = [np.array([0, 0]), np.array([x, y])]
                lines.append(segment)

        # Update line segments
        self.laser_LineCollection.set_segments(lines)

        # Apply transform to position the LineCollection based on current lidar origin (2D only)
        if not isinstance(ax, Axes3D):  # 2D case
            lidar_x = self.lidar_origin[0, 0]
            lidar_y = self.lidar_origin[1, 0]
            lidar_theta = (
                self.lidar_origin[2, 0] if self.lidar_origin.shape[0] > 2 else 0
            )

            # Create transform: rotate by lidar orientation, then translate to lidar position
            trans = (
                mtransforms.Affine2D().rotate(lidar_theta).translate(lidar_x, lidar_y)
                + ax.transData
            )
            self.laser_LineCollection.set_transform(trans)

    def step_plot(self):
        """
        Public method to update the lidar visualization, calls _step_plot.
        """
        self._step_plot()

    def set_laser_color(
        self, laser_indices, laser_color: str = "blue", alpha: float = 0.3
    ):
        """
        Set a specific color of the selected lasers.

        Args:
            laser_indices (list): The indices of the lasers to set the color.
            laser_color (str): The color to set the selected lasers. Default is 'blue'.
            alpha (float): The transparency of the lasers. Default is 0.3.
        """

        current_color = [self.color] * self.number
        current_alpha = [self.alpha] * self.number

        for index in laser_indices:
            if index < self.number:
                current_color[index] = laser_color
                current_alpha[index] = alpha

        self.laser_LineCollection.set_color(current_color)
        self.laser_LineCollection.set_alpha(current_alpha)

    def plot_clear(self):
        """
        Clear the plot elements from the axis.
        """
        [patch.remove() for patch in self.plot_patch_list]
        [line.pop(0).remove() for line in self.plot_line_list]
        [text.remove() for text in self.plot_text_list]

        self.plot_patch_list = []
        self.plot_line_list = []
        self.plot_text_list = []

    def scan_to_pointcloud(self):
        """
        Convert the Lidar scan data to a point cloud.

        Returns:
            np.ndarray: Point cloud (2xN).
        """
        point_cloud = []

        ranges = self.range_data
        angles = np.linspace(self.angle_min, self.angle_max, len(ranges))

        for i in range(len(ranges)):
            scan_range = ranges[i]
            angle = angles[i]

            if scan_range < (self.range_max - 0.02):
                point = np.array([[scan_range * cos(angle)], [scan_range * sin(angle)]])
                point_cloud.append(point)

        if len(point_cloud) == 0:
            return None

        return np.hstack(point_cloud)
