#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt

# LQR gain matrices (these are predefined for simplicity)
Q = np.diag([10.0, 10.0, 100.0, 1.0])  # State cost (penalize deviation in position, orientation)
R = np.diag([1.0, 1.0])  # Control input cost (penalize large control inputs)

# Predefined LQR gain matrix (K)
K = np.array([[1.0, 0.0, 0.0, 0.0],  # Linear velocity
              [0.0, 0.0, 1.0, 0.0]])  # Angular velocity

# Set the desired goal position (Point B) [x_goal, y_goal, theta_goal]
goal = np.array([2.0, 2.0, 0.0, 0.0])  # Example goal: x=2.0, y=2.0, theta=0.0

# Initialize lists to store control inputs and errors
control_inputs = []
errors = []

# Maximum angular velocity (rad/s) to limit excessive rotation
MAX_ANGULAR_VEL = 1.0  # Adjust this value to your needs

def lqr_control(state):
    """
    Compute the LQR control input based on the state.
    :param state: current state of the robot [x, y, theta, v]
    :return: control input [linear_velocity, angular_velocity]
    """
    error = state - goal  # Error between current state and goal
    errors.append(error)  # Store error for plotting

    # Control law (u = -K * error)
    control_input = -np.dot(K, error)  
    control_inputs.append(control_input)  # Store control input for plotting

    linear_velocity = control_input[0]
    angular_velocity = control_input[1]

    # Limit angular velocity to avoid excessive spinning
    angular_velocity = np.clip(angular_velocity, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)

    return linear_velocity, angular_velocity

def odom_callback(msg):
    """
    Callback to process odometry data.
    """
    # Extract position and velocity from the odometry message
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    # Convert quaternion to Euler angles to get theta (yaw)
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, theta) = euler_from_quaternion(orientation_list)

    # Velocity (linear velocity)
    v = msg.twist.twist.linear.x

    # Current state of the robot: [x, y, theta, v]
    current_state = np.array([x, y, theta, v])

    # Compute the LQR control input
    linear_vel, angular_vel = lqr_control(current_state)

    # Publish control input to cmd_vel
    cmd_vel = Twist()
    cmd_vel.linear.x = linear_vel
    cmd_vel.angular.z = angular_vel
    pub.publish(cmd_vel)

def plot_control_inputs():
    """
    Plot the control inputs (linear and angular velocity) over time.
    """
    if len(control_inputs) > 0:
        # Extract control inputs
        linear_velocities = [u[0] for u in control_inputs]
        angular_velocities = [u[1] for u in control_inputs]

        # Plot the control inputs
        plt.figure()
        plt.subplot(2, 1, 1)
        plt.plot(linear_velocities, label='Linear Velocity')
        plt.xlabel('Time Step')
        plt.ylabel('Linear Velocity')
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(angular_velocities, label='Angular Velocity')
        plt.xlabel('Time Step')
        plt.ylabel('Angular Velocity')
        plt.legend()

        plt.show()

if __name__ == '__main__':
    rospy.init_node('lqr_controller')

    # Publisher for velocity commands
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Subscriber for odometry data
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Run the node and make the TurtleBot move
    rospy.spin()

    # After the node finishes, plot the control inputs
    plot_control_inputs()
