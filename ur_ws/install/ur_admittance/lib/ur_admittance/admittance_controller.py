#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import WrenchStamped, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class AdmittanceController(Node):
    def __init__(self):
        super().__init__('admittance_controller')

        # Declare parameters
        self.declare_parameter('mass_matrix', [1.0, 1.0, 1.0, 0.5, 0.5, 0.5])
        self.declare_parameter('damping_matrix', [50.0, 50.0, 50.0, 20.0, 20.0, 20.0])
        self.declare_parameter('stiffness_matrix', [0.0, 0.0, 100.0, 10.0, 10.0, 10.0])
        self.declare_parameter('force_deadband', 2.0)
        self.declare_parameter('torque_deadband', 0.1)
        self.declare_parameter('max_velocity', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        
        # Get parameters
        self.mass = np.diag(self.get_parameter('mass_matrix').value)
        self.damping = np.diag(self.get_parameter('damping_matrix').value)
        self.stiffness = np.diag(self.get_parameter('stiffness_matrix').value)
        self.force_deadband = self.get_parameter('force_deadband').value
        self.torque_deadband = self.get_parameter('torque_deadband').value
        self.max_velocity = np.array(self.get_parameter('max_velocity').value)
        self.dt = 0.01  # Control loop period
        
        # Joint names for the UR3e
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Current state
        self.current_position = np.zeros(6)
        self.current_velocity = np.zeros(6)
        self.desired_position = np.zeros(6)
        self.ft_reading = np.zeros(6)
        self.initial_position = None

        # Initialize subscribers
        self.ft_sub = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.ft_callback,
            10)
            
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Initialize publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)

        # Create timer for control loop
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('Admittance Controller initialized')

    def ft_callback(self, msg):
        # Get raw force/torque readings
        forces = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        torques = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        
        # Apply deadband with smooth transition
        force_mask = abs(forces) > self.force_deadband
        torque_mask = abs(torques) > self.torque_deadband
        
        forces = np.where(force_mask, 
                         forces - np.sign(forces) * self.force_deadband, 
                         np.zeros_like(forces))
        torques = np.where(torque_mask, 
                          torques - np.sign(torques) * self.torque_deadband, 
                          np.zeros_like(torques))
        
        # Filter the readings with exponential smoothing
        alpha = 0.3  # Smoothing factor
        self.ft_reading = alpha * np.concatenate([forces, torques]) + (1 - alpha) * self.ft_reading

    def joint_state_callback(self, msg):
        # Get the positions of the UR3e joints
        positions = []
        velocities = []
        for joint_name in self.joint_names:
            idx = msg.name.index(joint_name)
            positions.append(msg.position[idx])
            if msg.velocity:  # Check if velocities are available
                velocities.append(msg.velocity[idx])
            else:
                velocities.append(0.0)
        
        self.current_position = np.array(positions)
        self.current_velocity = np.array(velocities)
        
        # Store initial position if not set
        if self.initial_position is None:
            self.initial_position = self.current_position.copy()
            self.desired_position = self.current_position.copy()

    def control_loop(self):
        if self.initial_position is None:
            return

        # Admittance control law
        # M(ddx) + D(dx) + K(x - x0) = F_ext
        # where x is the position, M is mass, D is damping, K is stiffness
        
        # Calculate position error from equilibrium
        pos_error = self.current_position - self.initial_position
        
        # Calculate acceleration from admittance equation
        # Using matrix operations for better numerical stability
        accel = np.linalg.solve(
            self.mass,
            self.ft_reading - 
            self.damping @ self.current_velocity - 
            self.stiffness @ pos_error
        )
        
        # Update velocity using semi-implicit Euler integration
        new_velocity = self.current_velocity + accel * self.dt
        
        # Apply velocity limits
        new_velocity = np.clip(new_velocity, -self.max_velocity, self.max_velocity)
        
        # Update desired position
        self.desired_position = self.current_position + new_velocity * self.dt
        self.current_velocity = new_velocity
        
        # Create and publish trajectory command
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.desired_position.tolist()
        point.velocities = self.current_velocity.tolist()
        point.accelerations = accel.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.dt * 1e9)
        
        trajectory_msg.points.append(point)
        self.trajectory_pub.publish(trajectory_msg)

def main():
    rclpy.init()
    controller = AdmittanceController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        # Update force-torque readings
        self.ft_reading[0] = msg.wrench.force.x
        self.ft_reading[1] = msg.wrench.force.y
        self.ft_reading[2] = msg.wrench.force.z
        self.ft_reading[3] = msg.wrench.torque.x
        self.ft_reading[4] = msg.wrench.torque.y
        self.ft_reading[5] = msg.wrench.torque.z

    def joint_state_callback(self, msg):
        if len(msg.actual.positions) > 0:
            self.current_position = np.array(msg.actual.positions)
            self.current_velocity = np.array(msg.actual.velocities)

    def control_loop(self):
        # Admittance control law
        # F = M*ddx + D*dx
        # ddx = (F - D*dx)/M
        accel = (self.ft_reading - self.damping * self.current_velocity) / self.mass
        
        # Update velocity and position
        self.current_velocity += accel * self.dt
        new_position = self.current_position + self.current_velocity * self.dt

        # Create and publish trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                              'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        point = JointTrajectoryPoint()
        point.positions = new_position.tolist()
        point.velocities = self.current_velocity.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.dt * 1e9)

        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = AdmittanceController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
