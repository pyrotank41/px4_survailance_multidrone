

# mavsdk packages
# import asyncio
# import mavsdk
# from mavsdk import System

# drone = System()


# ros pkgs impoort
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from rclpy.qos import qos_profile_sensor_data

class GimbleTrackerNode(Node):

    def __init__(self):
        super().__init__('gimble_tracking_node')

        #declare and read parameters
        self.declare_parameter('aruco_markers_topic', 
                                '/drone/ros2_aruco/aruco_markers')
        aruco_markers_topic = self.get_parameter('aruco_markers_topic').get_parameter_value().string_value
        # Set up subscriptions
        self.create_subscription(ArucoMarkers,
                                aruco_markers_topic,
                                self.marker_callback,  qos_profile_sensor_data)
    
    def marker_callback(self, marker_msg):
        
        ### mesage format
        """
            std_msgs/Header header
            int64[] marker_ids
            geometry_msgs/Pose[] poses
                Array of the following:
                geometry_msgs/Point position
                geometry_msgs/Quaternion orientation
        """
            
            
        id=5
        id_index = 0

        print(marker_msg.poses[id_index].position.x, end=",")
        print(marker_msg.poses[id_index].position.y, end=",")
        print(marker_msg.poses[id_index].position.y)


def main():
    rclpy.init()
    node = GimbleTrackerNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()