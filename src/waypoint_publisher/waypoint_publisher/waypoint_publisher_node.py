import rclpy
from rclpy.node import Node     
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush
from mavros_msgs.srv import CommandBool

import waypoint_publisher.modify_waypoints as modify_waypoints

class WaypointPublisherNode(Node):
    def __init__(self):
        super().__init__('waypoint_publisher_node')

        self.get_logger().info('creating waypoint_publisher_node')

        self.mission_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        # 63.3199076
        # 10.2710034
        self.declare_parameter('takeoff_land_lat', 38.316)
        self.declare_parameter('takeoff_land_long', -76.549)
        self.declare_parameter('above_land_height', 1.0)
        self.declare_parameter('travel_height', 10.)

        self.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        self.takeoff_land_lat = self.get_parameter('takeoff_land_lat').get_parameter_value().double_value
        self.takeoff_land_long = self.get_parameter('takeoff_land_long').get_parameter_value().double_value
        self.above_land_height = self.get_parameter('above_land_height').get_parameter_value().double_value
        self.travel_height = self.get_parameter('travel_height').get_parameter_value().double_value

        self.frame = Waypoint.FRAME_GLOBAL_REL_ALT

        self.load_geofence()
        self.load_waypoints()
        
    
    def load_geofence(self):
        self.geoFencePoints = self.load_from_file("src/waypoint_publisher/waypoint_publisher/waypoints/gf.txt")
 
    def load_waypoints(self):
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

        # takeoff waypoint
        takeoff_wp = Waypoint()
        takeoff_wp.frame = self.frame
        takeoff_wp.command = 22
        takeoff_wp.autocontinue = True
        takeoff_wp.x_lat = self.takeoff_land_lat
        takeoff_wp.y_long = self.takeoff_land_long
        takeoff_wp.z_alt = self.travel_height
        self.initial_waypoints.append(takeoff_wp)

        self.get_logger().info('takeoff waypoint initzialised')


        #Loading Waypoints from file
        ingress_waypoint_tuples = self.load_from_file("src/waypoint_publisher/waypoint_publisher/waypoints/wp.txt")
        
        # Convert the border points to east north up
        border_points_enu = [modify_waypoints.convert_gps_to_local_enu(point) for point in self.geoFencePoints]

        # Convert the waypoints to east north up coordinates (But also add the start waypoint)
        waypoints_enu = [modify_waypoints.convert_gps_to_local_enu(point) for point in [(self.takeoff_land_lat, self.takeoff_land_long)] + ingress_waypoint_tuples]

        # Calculate a new path that stays within the border, with a tolerance of 15 meters
        conforming_path = modify_waypoints.path_within_border(border_points_enu, waypoints_enu, 15)

        if conforming_path is None:
            self.get_logger().error("Some waypoints are outside the GeoFence! Continuing without modifying waypoints")
            modified_ingress_waypoint_tuples = ingress_waypoint_tuples
        else:
            # modify_waypoints.display(border_points_enu, waypoints_enu, conforming_path)
            # Remove the start waypoint
            conforming_path = conforming_path[1:]

            # Convert from east north up back to gps
            modified_ingress_waypoint_tuples = [modify_waypoints.convert_local_enu_to_gps(point) for point in conforming_path]


        self.ingress_waypoints = []
        #Creating mavros waypoint msgs
        for i in range(len(modified_ingress_waypoint_tuples)):
            wp = Waypoint()
            wp.frame = self.frame
            wp.command = 16
            wp.autocontinue = True
            wp.x_lat = modified_ingress_waypoint_tuples[i][0]
            wp.y_long = modified_ingress_waypoint_tuples[i][1]
            wp.z_alt = self.travel_height
            self.ingress_waypoints.append(wp)
            self._logger.info(f'{modified_ingress_waypoint_tuples[i][0]},{modified_ingress_waypoint_tuples[i][1]}')
        

        #Push waypoints 
        self.waypoints = self.initial_waypoints + self.ingress_waypoints
        self.get_logger().info(f'{len(self.waypoints)}')
        self.push_waypoints(self.waypoints,0)



        
    def load_from_file(self, filePath) -> list:
        with open(filePath, "r") as waypoints_file:
            self.wp_data = waypoints_file.read().splitlines()
            
        waypoint_tuples = []
        for data in self.wp_data:
            self._logger.info(f'{data}')
            data = data.split(",")

            waypoint_tuples.append((float(data[0].strip()), float(data[1].strip())))

        return waypoint_tuples

    def push_waypoints(self, waypoints: list, start_index=0) -> WaypointPush.Response:
        #checking what type the data is        
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

        response = future.result()

        self.get_logger().info(f'waypoint push resp: {response.success}')

        return response
        
            
def main(args = None):
    rclpy.init(args=args)

    waypoint_node = WaypointPublisherNode()
    rclpy.spin(waypoint_node)
    rclpy.shutdown()