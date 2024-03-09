
import os
import carb
import numpy as np
import struct

from stride.simulator.backends.backend import Backend
from omni.isaac.core.utils.extensions import disable_extension, enable_extension

# # Perform some checks, because Isaac Sim some times does not play nice when using ROS/ROS2
disable_extension("omni.isaac.ros_bridge")
enable_extension("omni.isaac.ros2_bridge-humble")

# Inform the user that now we are actually import the ROS2 dependencies
# Note: we are performing the imports here to make sure that ROS2 extension was load correctly
import rclpy  # pylint: disable=wrong-import-position
from std_msgs.msg import Float64  # pylint: disable=unused-import, wrong-import-position
from sensor_msgs.msg import (  # pylint: disable=unused-import, wrong-import-position
    Imu, PointCloud2, PointField, MagneticField, NavSatFix, NavSatStatus
)
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped  # pylint: disable=wrong-import-position


# set environment variable to use ROS2
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
os.environ["ROS_DOMAIN_ID"] = "15"

class ROS2Backend(Backend):
    """
    A class representing the ROS2 backend for the simulation.

    Args:
        node_name (str): The name of the ROS2 node.

    Attributes:
        node: The ROS2 node object.
        pose_pub: Publisher for the state of the vehicle in ENU.
        twist_pub: Publisher for the state of the vehicle's twist in ENU.
        twist_inertial_pub: Publisher for the state of the vehicle's inertial twist.
        accel_pub: Publisher for the state of the vehicle's acceleration.
        imu_pub: Publisher for the IMU sensor data.

    Methods:
        update(dt: float): Updates the state of the backend and the information being sent/received
                            from the communication interface.
        update_imu_data(data): Updates the IMU sensor data.
        update_sensor(sensor_type: str, data): Handles the receival of sensor data.
        update_state(state): Handles the receival of the state of the vehicle.
    """

    def __init__(self, node_name: str):
        """
        Initializes the ROS2Backend.

        Args:
            node_name (str): The name of the ROS2 node.
        """
        super().__init__()

        # Start the actual ROS2 setup here
        if not rclpy.ok():  # Check if the ROS2 context is already initialized
            rclpy.init()
        self.node = rclpy.create_node(node_name)



        # Create publishers for the state of the vehicle in ENU
        self.pose_pub = self.node.create_publisher(PoseStamped, node_name + "/state/pose", 10)
        self.twist_pub = self.node.create_publisher(TwistStamped, node_name + "/state/twist", 10)
        self.twist_inertial_pub = self.node.create_publisher(TwistStamped, node_name + "/state/twist_inertial", 10)
        self.accel_pub = self.node.create_publisher(AccelStamped, node_name + "/state/accel", 10)

        # Create publishers for some sensor data
        self.imu_pub = self.node.create_publisher(Imu, node_name + "/sensors/imu", 10)
        self.point_cloud_pub = self.node.create_publisher(PointCloud2, node_name + "/sensors/points", 10)

    def update(self, dt: float):
        """
        Method that when implemented, should be used to update the state of the backend and the information being
        sent/received from the communication interface. This method will be called by the simulation on every physics
        step.

        Args:
            dt (float): The time step for the update.
        """

        # In this case, do nothing as we are sending messages as soon as new data arrives from the sensors and state
        # and updating the reference for the thrusters as soon as receiving from ROS2 topics
        # Just poll for new ROS2 messages in a non-blocking way
        rclpy.spin_once(self.node, timeout_sec=0)

    def update_imu_data(self, data):
        """
        Updates the IMU sensor data.

        Args:
            data: The IMU sensor data.
        """

        msg = Imu()

        # Update the header
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link_frd"

        # Update the angular velocity (NED + FRD)
        msg.angular_velocity.x = data["angular_velocity"][0]
        msg.angular_velocity.y = data["angular_velocity"][1]
        msg.angular_velocity.z = data["angular_velocity"][2]

        # Update the linear acceleration (NED)
        msg.linear_acceleration.x = data["linear_acceleration"][0]
        msg.linear_acceleration.y = data["linear_acceleration"][1]
        msg.linear_acceleration.z = data["linear_acceleration"][2]

        # Publish the message with the current imu state
        self.imu_pub.publish(msg)

    def update_lidar_data(self, data):
        """
        Updates the Lidar sensor data.

        Args:
            data: The Lidar sensor data.
        """

        msg = PointCloud2()

        # Flatten LiDAR data
        points_flat = np.array(data["points"]).reshape(-1, 3)  # Adjust based on your data's structure

        # Create a PointCloud2 message
        msg = PointCloud2()
        # Update the header
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link_frd"
        msg.height = 1  # Unorganized point cloud
        msg.width = len(points_flat)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12  # Float32, x, y, z
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True  # No invalid points
        buffer = []

        # Populate the message with your LiDAR data
        for x, y, z in points_flat:
            buffer += [struct.pack("fff", x, y, z)]

        msg.data = b"".join(buffer)

        # Publish the message with the current lidar state
        self.point_cloud_pub.publish(msg)

    def update_sensor(self, sensor_type: str, data):
        """
        Method that when implemented, should handle the receival of sensor data.

        Args:
            sensor_type (str): The type of the sensor.
            data: The sensor data.
        """

        if sensor_type == "Imu":
            self.update_imu_data(data)
        elif sensor_type == "Lidar":
            self.update_lidar_data(data)
        else:
            carb.log_warn(f"Sensor type {sensor_type} is not supported by the ROS2 backend.")
            pass

    def update_state(self, state):
        """
        Method that when implemented, should handle the receival of the state of the vehicle using this callback.

        Args:
            state: The state of the vehicle.
        """

        pose = PoseStamped()
        twist = TwistStamped()
        twist_inertial = TwistStamped()
        accel = AccelStamped()

        # Update the header
        pose.header.stamp = (self.node.get_clock().now().to_msg())  # TODO: fill time when the state was measured.
        twist.header.stamp = pose.header.stamp
        twist_inertial.header.stamp = pose.header.stamp
        accel.header.stamp = pose.header.stamp

        pose.header.frame_id = "map"
        twist.header.frame_id = "base_link"
        twist_inertial.header.frame_id = "base_link"
        accel.header.frame_id = "base_link"

        # Fill the position and attitude of the vehicle in ENU.
        pose.pose.position.x = state.position[0]
        pose.pose.position.y = state.position[1]
        pose.pose.position.z = state.position[2]

        pose.pose.orientation.x = state.attitude[0]
        pose.pose.orientation.y = state.attitude[1]
        pose.pose.orientation.z = state.attitude[2]
        pose.pose.orientation.w = state.attitude[3]

        # Fill the linear and angular velocities in the body frame of the vehicle.
        twist.twist.linear.x = state.linear_body_velocity[0]
        twist.twist.linear.y = state.linear_body_velocity[1]
        twist.twist.linear.z = state.linear_body_velocity[2]

        twist.twist.angular.x = state.angular_velocity[0]
        twist.twist.angular.y = state.angular_velocity[1]
        twist.twist.angular.z = state.angular_velocity[2]

        # Fill the linear velocity of the vehicle in the inertial frame.
        twist_inertial.twist.linear.x = state.linear_velocity[0]
        twist_inertial.twist.linear.y = state.linear_velocity[1]
        twist_inertial.twist.linear.z = state.linear_velocity[2]

        # Fill the linear acceleration in the inertial frame
        accel.accel.linear.x = state.linear_acceleration[0]
        accel.accel.linear.y = state.linear_acceleration[1]
        accel.accel.linear.z = state.linear_acceleration[2]

        # Publish the messages containing the state of the vehicle.
        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)
        self.twist_inertial_pub.publish(twist_inertial)
        self.accel_pub.publish(accel)

    # def check_ros_extension(self):
    #     """
    #     Method that checks which ROS extension is installed.
    #     """

    #     # Get the handle for the extension manager
    #     extension_manager = omni.kit.app.get_app().get_extension_manager()

    #     version = ""

    #     if self._ext_manager.is_extension_enabled("omni.isaac.ros_bridge"):
    #         version = "ros"
    #     elif self._ext_manager.is_extension_enabled("omni.isaac.ros2_bridge"):
    #         version = "ros2"
    #     else:
    #         carb.log_warn("Neither extension 'omni.isaac.ros_bridge' nor 'omni.isaac.ros2_bridge' is enabled")
