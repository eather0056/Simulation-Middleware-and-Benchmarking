import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile

class VFFColor:
    RED = 0
    GREEN = 1
    BLUE = 2

class VFFVectors:
    def __init__(self):
        self.attractive = [1.0, 0.0]
        self.repulsive = [0.0, 0.0]
        self.result = [0.0, 0.0]

class AvoidanceNode(Node):
    def __init__(self):
        super().__init__("avoidance_vff")
        qos = QoSProfile(depth=100)
        
        self.vel_pub = self.create_publisher(Twist, "output_vel", qos)
        self.vff_debug_pub = self.create_publisher(MarkerArray, "vff_debug", qos)
        self.scan_sub = self.create_subscription(
            LaserScan, "input_scan", self.scan_callback, qos
        )

        self.timer = self.create_timer(0.05, self.control_cycle)
        self.last_scan = None

    def scan_callback(self, msg):
        self.last_scan = msg

    def control_cycle(self):
        # Skip cycle if no valid recent scan available
        if self.last_scan is None or (self.get_clock().now() - rclpy.time.Time.from_msg(self.last_scan.header.stamp)).nanoseconds > rclpy.time.Duration(seconds=1.0).nanoseconds:
            return

        # Get VFF vectors
        vff = self.get_vff(self.last_scan)

        # Use result vector to calculate output speed
        v = vff.result
        angle = math.atan2(v[1], v[0])
        module = math.sqrt(v[0] ** 2 + v[1] ** 2)

        # Create output message, controlling speed limits
        vel = Twist()
        vel.linear.x = max(0.0, min(module, 0.3))  # truncate linear vel to [0.0, 0.3] m/s
        vel.angular.z = max(-0.5, min(angle, 0.5))  # truncate rotation vel to [-0.5, 0.5] rad/s

        self.vel_pub.publish(vel)

        # Produce debug information, if any interested
        if self.vff_debug_pub.get_subscription_count() > 0:
            self.vff_debug_pub.publish(self.get_debug_vff(vff))

    def get_vff(self, scan):
        # This is the obstacle radius in which an obstacle affects the robot
        OBSTACLE_DISTANCE = 0.3

        # Initialize vectors
        vff_vector = VFFVectors()
        min_idx = min(range(len(scan.ranges)), key=lambda i: scan.ranges[i])
        distance_min = scan.ranges[min_idx]

        # If the obstacle is in the area that affects the robot, calculate repulsive vector
        if distance_min < OBSTACLE_DISTANCE:
            angle = scan.angle_min + scan.angle_increment * min_idx
            opposite_angle = angle + math.pi
            complementary_dist = OBSTACLE_DISTANCE - distance_min
            complementary_dist *= 2  # Increase the factor to strengthen repulsion


            # Get cartesian (x, y) components from polar (angle, distance)
            vff_vector.repulsive[0] = math.cos(opposite_angle) * complementary_dist
            vff_vector.repulsive[1] = math.sin(opposite_angle) * complementary_dist

        # Calculate resulting vector by adding attractive and repulsive vectors
        vff_vector.result[0] = (vff_vector.repulsive[0] + vff_vector.attractive[0])
        vff_vector.result[1] = (vff_vector.repulsive[1] + vff_vector.attractive[1])

        return vff_vector

    def get_debug_vff(self, vff_vectors):
        marker_array = MarkerArray()
        marker_array.markers.append(self.make_marker(vff_vectors.attractive, VFFColor.BLUE))
        marker_array.markers.append(self.make_marker(vff_vectors.repulsive, VFFColor.RED))
        marker_array.markers.append(self.make_marker(vff_vectors.result, VFFColor.GREEN))
        return marker_array

    def make_marker(self, vector, vff_color):
        marker = Marker()
        marker.header.frame_id = "robot1/base_footprint"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start = Point(x=0.0, y=0.0)
        end = Point(x=vector[0], y=vector[1])
        marker.points = [start, end]

        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.color.a = 1.0

        if vff_color == VFFColor.RED:
            marker.id = 0
            marker.color.r = 1.0
        elif vff_color == VFFColor.GREEN:
            marker.id = 1
            marker.color.g = 1.0
        elif vff_color == VFFColor.BLUE:
            marker.id = 2
            marker.color.b = 1.0

        return marker

def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()