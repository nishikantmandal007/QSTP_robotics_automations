import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class TurtlebotPIDController:
    def __init__(self):
        rospy.init_node('turtlebot_pid_controller', anonymous=True)
        
        self.goal_x = 5.0  # Example goal x-coordinate
        self.goal_y = 5.0  # Example goal y-coordinate
        
        self.Kp_linear = 0.5
        self.Ki_linear = 0.0
        self.Kd_linear = 0.1
        
        self.Kp_angular = 0.5
        self.Ki_angular = 0.0
        self.Kd_angular = 0.1
        
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        
        self.accumulated_linear_error = 0.0
        self.accumulated_angular_error = 0.0
        
        self.max_linear_velocity = 0.5
        self.max_angular_velocity = 1.0
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                      1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
        
    def calculate_control(self):
        linear_error = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        angular_error = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x) - self.current_yaw
        
        delta_linear_error = linear_error - self.prev_linear_error
        delta_angular_error = angular_error - self.prev_angular_error
        
        self.accumulated_linear_error += linear_error
        self.accumulated_angular_error += angular_error
        
        linear_velocity = self.Kp_linear * linear_error + self.Ki_linear * self.accumulated_linear_error + self.Kd_linear * delta_linear_error
        angular_velocity = self.Kp_angular * angular_error + self.Ki_angular * self.accumulated_angular_error + self.Kd_angular * delta_angular_error
        
        linear_velocity = min(self.max_linear_velocity, max(-self.max_linear_velocity, linear_velocity))
        angular_velocity = min(self.max_angular_velocity, max(-self.max_angular_velocity, angular_velocity))
        
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.prev_linear_error = linear_error
        self.prev_angular_error = angular_error
        
    def run(self):
        rate = rospy.Rate(10)  # Hz
        
        while not rospy.is_shutdown():
            self.calculate_control()
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = TurtlebotPIDController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

