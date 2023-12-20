#!/usr/bin/env python3
import rospy
from node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from math import atan2, pi
import math
from numpy import nanmin
from tf2_msgs.msg import TFMessage
from tf import TransformListener, ExtrapolationException, LookupException
import time

move_tolerance = 0.1
scan_tolerance_front = 0.4
scan_tolerance_side = 0.3
rotate_tolerance = 0.004
linear_velocity = rospy.get_param("/path_follower_node/linear_velocity", 0.25)
angular_velocity = rospy.get_param("/path_follower_node/angular_velocity", 1.5)


class PathFollower:
    """Follows the path obtained to one of its topics"""

    def __init__(self):
        self.robot_position = None  # Set from planner
        self.path_deviation = 0.0  # Used to avoid obstacles

        # self.planner = planner
        self.goal = PoseStamped()

        self.following_path = []

        self.is_shutdown_initiated = False
        self.is_moving = False
        self.is_obstacle_ahead = False

        self.goal_tolerance = rospy.get_param("~goal_tolerance", default=0.1)

        self.transform_listener = TransformListener()

        self.new_path_arrived = False

        time.sleep(2)  # wait for transforms to be available

        self.transform_listener.waitForTransform(
            "/base_link", "/map", rospy.Time(), timeout=rospy.Duration(5)
        )

        #! Subcribers
        self.mover_subscriber = rospy.Subscriber("/path", Path, self.path_callback)
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.motion_stop_subscriber = rospy.Subscriber(
            "/stop_motion", Bool, self.stop_motion_callback
        )
        self.start_subscriber = rospy.Subscriber(
            "/tf", TFMessage, self.robot_pose_callback
        )
        self.goal_subscriber = rospy.Subscriber(
            "/goal_controller", PoseStamped, self.goal_callback
        )

        #! Publishers
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.motion_publisher = rospy.Publisher("/robot_is_moving", Bool, queue_size=10)
        self.goal_reached_publisher = rospy.Publisher(
            "/goal_reached", Bool, queue_size=10
        )

    def goal_callback(self, data: PoseStamped) -> bool:
        """Obtain goal from query"""
        self.goal = Node.from_pose(data.pose)

    def robot_pose_callback(self, data: TFMessage):
        """Listens to robot current pose"""
        try:
            position, quaternion = self.transform_listener.lookupTransform(
                "/map", "/base_link", rospy.Time()
            )
        except (ExtrapolationException, LookupException):
            return

        self.robot_position = Node.from_tf(position, quaternion)

    def stop_motion_callback(self, stop_motion: Bool):
        """Listens to a flag if the motions has to be stopped"""
        if stop_motion:
            self.stop_moving()
            self.initialize_stop()

    def path_callback(self, path: Path):
        """Listens to obtained solution path from path planner"""

        self.new_path_arrived = True
        node_path = []

        nearest_node = 0
        distance = float("inf")

        for i in range(0, len(path.poses)):
            node = Node.from_pose(path.poses[i].pose)
            node_path.append(node)
            node_distance = node.calculate_distance(self.robot_position)
            if node_distance < distance:
                distance = node_distance
                nearest_node = i

        node_path = node_path[nearest_node:]
        self.following_path = node_path
        self.is_shutdown_initiated = False

    def scan_callback(self, scan_data: LaserScan):
        """Listens to the scan obtained from the lidar"""
        if (
            nanmin(scan_data.ranges[0:10] + scan_data.ranges[350:360])
            < scan_tolerance_front
        ):
            self.is_obstacle_ahead = False
        elif nanmin(scan_data.ranges[11:165]) < scan_tolerance_side:
            self.path_deviation = 0
        elif nanmin(scan_data.ranges[195:349]) < scan_tolerance_side:
            self.path_deviation = 0
        else:
            self.path_deviation = 0

    def initialize_stop(self):
        self.is_shutdown_initiated = True

    def stop_moving(self):
        self.following_path = []
        self.is_shutdown_initiated = False
        self.is_moving = False
        self.velocity_publisher.publish(Twist())

    def follow_path(self):
        self.is_moving = True

        while len(self.following_path) > 0:
            if self.is_shutdown_initiated:
                self.stop_moving()
                return
            node_to_follow = self.following_path.pop(0)
            self.move_to_point(node_to_follow)

        if (
            math.sqrt(
                math.pow(self.goal.x - self.robot_position.x, 2)
                + math.pow(self.goal.y - self.robot_position.y, 2)
            )
            < self.goal_tolerance + 0.1
        ):
            self.velocity_publisher.publish(Twist())  # Stopping robot
            self.rotate_to_goal(self.goal)
            self.velocity_publisher.publish(Twist())

            self.goal_reached_publisher.publish(True)
            rospy.loginfo("Goal has been reached")
            self.is_moving = False

    def go_back(self):
        current_distance = 0.0
        vel_msg = Twist()
        vel_msg.linear.x = -linear_velocity
        t0 = rospy.Time().now().to_sec()
        loop_rate = rospy.Rate(1000)

        while current_distance < 0.4:
            if self.is_shutdown_initiated:
                self.is_obstacle_ahead = False
                return

            self.velocity_publisher.publish(vel_msg)
            t1 = rospy.Time().now().to_sec()
            current_distance = linear_velocity * (t1 - t0)
            loop_rate.sleep()

        self.is_obstacle_ahead = False

    def move_to_point(self, point: Node):
        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        loop_rate = rospy.Rate(1000)

        while self.robot_position.calculate_distance(point) > move_tolerance:
            if (
                self.is_shutdown_initiated
                or self.is_obstacle_ahead
                or self.new_path_arrived
            ):
                self.new_path_arrived = False
                return

            speed = angular_velocity * self.angular_difference(point)
            vel_msg.angular.z = min(angular_velocity, speed) + self.path_deviation

            self.velocity_publisher.publish(vel_msg)
            loop_rate.sleep()

    def rotate_to_goal(self, goal: Node):
        if isinstance(goal, PoseStamped):
            goal = Node.from_pose(goal.pose)

        vel_msg = Twist()
        loop_rate = rospy.Rate(1000)

        while abs(goal.theta - self.robot_position.theta) > rotate_tolerance:
            if self.is_shutdown_initiated:
                self.stop_moving()
                return

            speed = angular_velocity * (goal.theta - self.robot_position.theta)
            vel_msg.angular.z = min(angular_velocity, speed)
            self.velocity_publisher.publish(vel_msg)
            loop_rate.sleep()

        self.velocity_publisher.publish(Twist())

    def angular_difference(self, point: Node) -> float:
        angle = (
            atan2(point.y - self.robot_position.y, point.x - self.robot_position.x)
            - self.robot_position.theta
        )

        if angle <= -pi:  # Normalizing angle
            angle += 2 * pi
        elif angle > pi:
            angle -= 2 * pi

        return angle


if __name__ == "__main__":
    try:
        rospy.init_node("path_follower_node", anonymous=True)

        controller = PathFollower()

        while not rospy.is_shutdown():
            if len(controller.following_path) > 0:
                controller.follow_path()
            else:
                pass

    except rospy.ROSInterruptException:
        pass
    except ValueError:
        print("Error: not a number")
