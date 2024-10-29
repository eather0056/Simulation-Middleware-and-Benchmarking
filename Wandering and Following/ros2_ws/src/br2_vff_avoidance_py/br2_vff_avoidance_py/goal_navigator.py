import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
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

        # Declare goal parameters
        self.declare_parameter("goal_x", 0.8)
        self.declare_parameter("goal_y", 2.0)

        # Retrieve goal position from parameters
        self.goal_position = (
            self.get_parameter("goal_x").get_parameter_value().double_value,
            self.get_parameter("goal_y").get_parameter_value().double_value,
        )

        self.vel_pub = self.create_publisher(Twist, "output_vel", qos)
        self.vff_debug_pub = self.create_publisher(MarkerArray, "vff_debug", qos)
        self.scan_sub = self.create_subscription(
            LaserScan, "/robot1/scan", self.scan_callback, qos
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/robot1/odom", self.odom_callback, qos
        )

        self.timer = self.create_timer(0.05, self.control_cycle)
        self.last_scan = None
        self.stuck_timer = 0
        self.stuck_timeout = 50
        self.unstuck_mode = False
        self.unstuck_stage = 0

        # Initialize position and time tracking for unstuck behavior
        self.current_position = (0.0, 0.0)
        self.previous_position = None
        self.start_position = None
        self.start_time = None
        self.stuck_distance_threshold = 0.2

        # Goal-related parameters
        self.goal_reached = False
        self.goal_tolerance = 0.1  # Distance within which the goal is considered reached

    def scan_callback(self, msg):
        self.get_logger().info(f"Laser scan received with timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        self.last_scan = msg

    def odom_callback(self, msg):
        # Update the current position from odometry data
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Odometry position updated: {self.current_position}")

    def control_cycle(self):
        # Print retrieved goal position for verification
        self.get_logger().info(f"Goal Position: {self.goal_position}")

        # Skip cycle if no valid recent scan available or goal is reached
        if self.goal_reached:
            self.stop_robot()
            return

        if self.last_scan is None:
            self.get_logger().info("Waiting for laser scan data...")
            return

        # Bypass ROS time and rely on the existence of `self.last_scan`
        self.get_logger().info("Laser scan data is available; proceeding with control cycle.")



        # Check if the goal is reached
        if self.is_goal_reached():
            self.goal_reached = True
            self.get_logger().info("Goal reached!")
            self.stop_robot()
            return

        # Print distance to goal for monitoring
        distance_to_goal = self.distance_to_goal()
        self.get_logger().info(f"Distance to goal: {distance_to_goal:.2f} meters")

        # If in unstuck mode, perform unstuck maneuver
        if self.unstuck_mode:
            self.execute_unstuck_maneuver()
            return

        # Get VFF vectors for obstacle avoidance
        vff = self.get_vff(self.last_scan)

        # Calculate vector towards goal
        goal_vector = self.get_goal_vector()

        # Combine goal vector and avoidance vector with weights
        goal_weight = 2.0
        avoidance_weight = 0.8
        v = [
            goal_weight * goal_vector[0] + avoidance_weight * vff.result[0],
            goal_weight * goal_vector[1] + avoidance_weight * vff.result[1],
        ]
        
        angle = math.atan2(v[1], v[0])
        module = math.sqrt(v[0] ** 2 + v[1] ** 2)

        # Check if the robot is stuck by verifying position change
        if self.is_stuck():
            self.stuck_timer += 1
            self.get_logger().info("Robot appears to be stuck.")
        else:
            self.stuck_timer = 0

        # Activate "unstuck" mode if the stuck timer exceeds the threshold
        if self.stuck_timer >= self.stuck_timeout:
            self.unstuck_mode = True
            self.unstuck_stage = 0
            self.start_position = self.current_position
            self.stuck_timer = 0
            self.get_logger().info("Entering unstuck mode.")
            return

        # Adjust speed based on the distance to the goal for efficiency
        max_linear_speed = min(0.5, 0.1 + 0.4 * distance_to_goal)
        max_angular_speed = min(0.8, 0.2 + 0.6 * abs(angle))

        # Create output message, controlling speed limits
        # Create output message, controlling speed limits
        vel = Twist()
        vel.linear.x = max(0.0, min(module, max_linear_speed))
        vel.angular.z = max(-max_angular_speed, min(angle, max_angular_speed))

        self.get_logger().info(f"Publishing Twist command: linear.x={vel.linear.x}, angular.z={vel.angular.z}")
        self.vel_pub.publish(vel)

        # Publish debug information if there are any subscribers
        if self.vff_debug_pub.get_subscription_count() > 0:
            self.vff_debug_pub.publish(self.get_debug_vff(vff))

    def stop_robot(self):
        """Stop the robot by publishing zero velocity."""
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.vel_pub.publish(vel)

    def is_goal_reached(self):
        """Check if the robot has reached the goal position."""
        return self.distance_to_goal() <= self.goal_tolerance

    def distance_to_goal(self):
        """Calculate the distance from the robot to the goal position."""
        return math.sqrt(
            (self.goal_position[0] - self.current_position[0]) ** 2 +
            (self.goal_position[1] - self.current_position[1]) ** 2
        )

    def get_goal_vector(self):
        """Calculate the attractive vector pointing towards the goal."""
        goal_dx = self.goal_position[0] - self.current_position[0]
        goal_dy = self.goal_position[1] - self.current_position[1]
        distance = math.sqrt(goal_dx ** 2 + goal_dy ** 2)
        if distance > 0:
            return [goal_dx / distance, goal_dy / distance]
        return [0, 0]

    def is_stuck(self):
        """Check if the robot's position has not changed significantly over time."""
        if self.previous_position is None:
            self.previous_position = self.current_position
            return False

        # Calculate distance moved since the last cycle
        distance_moved = math.sqrt(
            (self.current_position[0] - self.previous_position[0]) ** 2 +
            (self.current_position[1] - self.previous_position[1]) ** 2
        )

        # Update previous position
        self.previous_position = self.current_position

        # If the distance moved is below the threshold, consider the robot stuck
        return distance_moved < self.stuck_distance_threshold

    def execute_unstuck_maneuver(self):
        vel = Twist()
        
        # Stage 0: Move backward at 0.2 m/s for 3 seconds
        if self.unstuck_stage == 0:
            if self.start_time is None:
                self.start_time = self.get_clock().now()
            
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed_time >= 1.0:
                self.unstuck_stage = 1
                self.start_time = None
            else:
                vel.linear.x = -0.2

        # Stage 1: Rotate 180 degrees
        elif self.unstuck_stage == 1:
            if self.start_time is None:
                self.start_time = self.get_clock().now()

            rotation_duration = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if rotation_duration >= 2.0:
                self.unstuck_mode = False
                self.unstuck_stage = 0
                self.start_time = None
            else:
                vel.angular.z = 1.0

        self.vel_pub.publish(vel)

    def get_vff(self, scan):
        OBSTACLE_DISTANCE = 0.5
        vff_vector = VFFVectors()
        min_idx = min(range(len(scan.ranges)), key=lambda i: scan.ranges[i])
        distance_min = scan.ranges[min_idx]

        if distance_min < OBSTACLE_DISTANCE:
            angle = scan.angle_min + scan.angle_increment * min_idx
            opposite_angle = angle + math.pi
            complementary_dist = OBSTACLE_DISTANCE - distance_min
            complementary_dist *= 2

            vff_vector.repulsive[0] = math.cos(opposite_angle) * complementary_dist
            vff_vector.repulsive[1] = math.sin(opposite_angle) * complementary_dist

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
