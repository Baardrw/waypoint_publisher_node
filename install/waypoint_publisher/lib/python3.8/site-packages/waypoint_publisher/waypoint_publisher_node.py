from operator import le
import rclpy
from rclpy.node import Node     
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import CommandBool

class WaypointPublisherNode(Node):
    def __init__(self):
        super().__init__('waypoint_publisher_node')

        self.get_logger().info('creating waypoint_publisher_node')

        self.mission_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        # 63.3199076
        # 10.2710034
        self.declare_parameter('takeoff_land_lat', 47.3977508)
        self.declare_parameter('takeoff_land_long', 8.5456073)
        self.declare_parameter('above_land_height', 1.0)
        self.declare_parameter('travel_height', 10.)

        self.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        self.takeoff_land_lat = self.get_parameter('takeoff_land_lat').get_parameter_value().double_value
        self.takeoff_land_long = self.get_parameter('takeoff_land_long').get_parameter_value().double_value
        self.above_land_height = self.get_parameter('above_land_height').get_parameter_value().double_value
        self.travel_height = self.get_parameter('travel_height').get_parameter_value().double_value

        self.frame = Waypoint.FRAME_GLOBAL_REL_ALT

        self.initialization_state()
        
 
    def initialization_state(self):
        self.get_logger().info('initzialising waypoints')

        self.initial_waypoints = []
        # first wp is always home for pxcopter
        mission_start_wp = Waypoint()
        mission_start_wp.frame = self.frame
        mission_start_wp.command = 16
        mission_start_wp.autocontinue = True
        mission_start_wp.x_lat = self.takeoff_land_lat
        mission_start_wp.y_long = self.takeoff_land_long
        mission_start_wp.z_alt = self.travel_height
        self.initial_waypoints.append(mission_start_wp)

        self.get_logger().info('home waypoint initzialised')

        # takeoff wp
        takeoff_wp = Waypoint()
        takeoff_wp.frame = self.frame
        takeoff_wp.command = 22
        takeoff_wp.autocontinue = True
        takeoff_wp.x_lat = self.takeoff_land_lat
        takeoff_wp.y_long = self.takeoff_land_long
        takeoff_wp.z_alt = self.travel_height
        self.initial_waypoints.append(takeoff_wp)

        self.get_logger().info('takeoff waypoint initzialised')


        #tentativley loading waypoints from file
        self.ingress_waypoint_tuples = self.load_waypoints_from_file()
        self.ingress_waypoints = []

        for i in range(len(self.ingress_waypoint_tuples)):
            wp = Waypoint()
            wp.frame = self.frame
            wp.command = 16
            wp.autocontinue = True
            wp.x_lat = self.ingress_waypoint_tuples[i][0]
            wp.y_long = self.ingress_waypoint_tuples[i][1]
            wp.z_alt = self.travel_height
            self.ingress_waypoints.append(wp)
            self._logger.info(f'{self.ingress_waypoint_tuples[i][0]},{self.ingress_waypoint_tuples[i][1]}')
        
        #publishing waypoints
        # self.waypoints = self.initial_waypoints + self.initial_waypoints
        # req = CommandBool.Request()
        # req.value = True

        # self.get_logger().info("1")
        # while not self.arm_client.wait_for_service(timeout_sec=1):
        #     self.get_logger().info(f'wait for mission push service timed out')

        # future = self.arm_client.call_async(req)
        # rclpy.spin_until_future_complete(self,future)
        # self.get_logger().info(f'{future}')

        self.waypoints = self.initial_waypoints + self.ingress_waypoints
        self.get_logger().info(f'{len(self.waypoints)}')
        self.push_waypoints(self.waypoints,0)

        
    def load_waypoints_from_file(self) -> list:
        with open("src/waypoint_publisher/waypoint_publisher/waypoints/suas.txt", "r") as waypoints_file:
            self.wp_data = waypoints_file.read().splitlines()
            
        waypoint_tuples = []
        for data in self.wp_data:
            self._logger.info(f'{data}')
            data = data.split(",")

            waypoint_tuples.append((float(data[0].strip()), float(data[1].strip())))

        return waypoint_tuples

    def push_waypoints(self, waypoints, start_index=0) -> WaypointPush.Response:
        self.get_logger().info(f'pushing {len(waypoints)} waypoints')
        req = WaypointPush.Request()
        req.start_index = start_index
        req.waypoints = waypoints
        self.get_logger().info(f'1')



        while not self.mission_push_client.wait_for_service(timeout_sec=1):
            self.get_logger().info(f'wait for mission push service timed out')

        self.get_logger().info(f'2')
        
        future = self.mission_push_client.call_async(req)
        rclpy.spin_until_future_complete(self,future)

        self.get_logger().info(f'3')

        self.get_logger().info(f'waypoint push resp: {future}')

        return future            
            
def main(args = None):
    rclpy.init(args=args)

    waypoint_node = WaypointPublisherNode()
    rclpy.spin(waypoint_node)
    rclpy.shutdown()