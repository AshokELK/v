#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import WrenchStamped, Twist, TransformStamped, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from scipy.spatial.transform import Rotation
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class SimpleAdmittanceController(Node):
    def __init__(self):
        super().__init__('simple_admittance_controller')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mass_matrix', [10.0, 10.0, 10.0, 2.0, 2.0, 2.0]),  # M matrix diagonal
                ('damping_matrix', [200.0, 200.0, 200.0, 20.0, 20.0, 20.0]),  # D matrix diagonal
                ('stiffness_matrix', [300.0, 300.0, 300.0, 30.0, 30.0, 30.0]),  # K matrix diagonal
                ('force_dead_zone', [2.0, 2.0, 2.0]),  # Minimum force in N
                ('torque_dead_zone', [0.1, 0.1, 0.1]),  # Minimum torque in Nm
                ('max_velocity', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),  # Maximum velocity in m/s and rad/s
                ('max_acceleration', [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]),  # Maximum acceleration in m/s^2 and rad/s^2
                ('control_rate', 100.0),  # Control loop rate in Hz
                ('base_link', 'base_link'),
                ('end_link', 'tool0'),
                ('interface_type', 'position')  # 'position' or 'velocity'
            ]
        )
        
        # Get parameters
        self.M = np.diag(self.get_parameter('mass_matrix').value)
        self.D = np.diag(self.get_parameter('damping_matrix').value)
        self.K = np.diag(self.get_parameter('stiffness_matrix').value)
        self.force_dead_zone = np.array(self.get_parameter('force_dead_zone').value)
        self.torque_dead_zone = np.array(self.get_parameter('torque_dead_zone').value)
        self.max_vel = np.array(self.get_parameter('max_velocity').value)
        self.max_acc = np.array(self.get_parameter('max_acceleration').value)
        self.control_rate = self.get_parameter('control_rate').value
        self.base_link = self.get_parameter('base_link').value
        self.end_link = self.get_parameter('end_link').value
        self.interface_type = self.get_parameter('interface_type').value
        
        # Robot joint names
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # State variables
        self.dt = 1.0 / self.control_rate
        self.current_pose = np.zeros(7)  # [x, y, z, qx, qy, qz, qw]
        self.current_velocity = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]
        self.wrench_external = np.zeros(6)  # [fx, fy, fz, tx, ty, tz]
        self.wrench_desired = np.zeros(6)  # Desired wrench
        
        # Integration variables for admittance equation
        self.delta_x = np.zeros(6)  # Position/orientation error
        self.delta_x_dot = np.zeros(6)  # Velocity error
        self.delta_x_prev = np.zeros(6)
        self.delta_x_dot_prev = np.zeros(6)
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Callback group for parallel processing
        self.callback_group = ReentrantCallbackGroup()
        
        # Create subscribers
        self.wrench_sub = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.wrench_callback,
            sensor_qos,
            callback_group=self.callback_group
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Create publishers based on interface type
        if self.interface_type == 'velocity':
            self.cmd_pub = self.create_publisher(
                Twist,
                '/cartesian_velocity_controller/command',
                10
            )
        else:  # position interface
            self.cmd_pub = self.create_publisher(
                Pose,
                '/cartesian_position_controller/command',
                10
            )
        
        # Timer for control loop
        self.timer = self.create_timer(
            self.dt,
            self.control_loop,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Simple Admittance Controller initialized')
    
    def wrench_callback(self, msg):
        # Convert wrench to base frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_link,
                msg.header.frame_id,
                rclpy.time.Time()
            )
            
            # Extract rotation matrix from transform
            q = [transform.transform.rotation.x,
                 transform.transform.rotation.y,
                 transform.transform.rotation.z,
                 transform.transform.rotation.w]
            rot = Rotation.from_quat(q)
            rot_matrix = rot.as_matrix()
            
            # Create wrench rotation matrix
            wrench_rot = np.zeros((6,6))
            wrench_rot[:3,:3] = rot_matrix
            wrench_rot[3:,3:] = rot_matrix
            
            # Transform wrench
            force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
            torque = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
            wrench = np.concatenate([force, torque])
            wrench_base = wrench_rot @ wrench
            
            # Apply dead zones
            force_base = wrench_base[:3]
            torque_base = wrench_base[3:]
            force_base = np.where(np.abs(force_base) < self.force_dead_zone, 0.0, force_base)
            torque_base = np.where(np.abs(torque_base) < self.torque_dead_zone, 0.0, torque_base)
            
            self.wrench_external = np.concatenate([force_base, torque_base])
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Transform lookup failed: {str(e)}')
    
    def joint_state_callback(self, msg):
        try:
            # Get current end-effector pose
            transform = self.tf_buffer.lookup_transform(
                self.base_link,
                self.end_link,
                rclpy.time.Time()
            )
            
            # Update pose
            self.current_pose[:3] = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]
            self.current_pose[3:] = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Transform lookup failed: {str(e)}')
    
    def compute_admittance(self):
        # Admittance equation: M(ddx) + D(dx) + K(x) = F_ext - F_des
        # Solved for acceleration: ddx = M^-1(F_ext - F_des - D(dx) - K(x))
        
        # Calculate net force
        F_net = self.wrench_external - self.wrench_desired
        
        # Calculate acceleration
        delta_x_ddot = np.linalg.solve(
            self.M,
            F_net - self.D @ self.delta_x_dot - self.K @ self.delta_x
        )
        
        # Semi-implicit Euler integration
        self.delta_x_dot = self.delta_x_dot_prev + delta_x_ddot * self.dt
        self.delta_x = self.delta_x_prev + self.delta_x_dot * self.dt
        
        # Apply velocity and acceleration limits
        self.delta_x_dot = np.clip(self.delta_x_dot, -self.max_vel, self.max_vel)
        delta_x_ddot = np.clip(delta_x_ddot, -self.max_acc, self.max_acc)
        
        # Update previous values
        self.delta_x_prev = self.delta_x.copy()
        self.delta_x_dot_prev = self.delta_x_dot.copy()
        
        return self.delta_x, self.delta_x_dot
    
    def control_loop(self):
        if np.all(self.current_pose == 0):  # Not initialized yet
            return
            
        # Compute admittance response
        delta_x, delta_x_dot = self.compute_admittance()
        
        if self.interface_type == 'velocity':
            # Publish velocity command
            cmd = Twist()
            cmd.linear.x = delta_x_dot[0]
            cmd.linear.y = delta_x_dot[1]
            cmd.linear.z = delta_x_dot[2]
            cmd.angular.x = delta_x_dot[3]
            cmd.angular.y = delta_x_dot[4]
            cmd.angular.z = delta_x_dot[5]
            self.cmd_pub.publish(cmd)
            
        else:  # position interface
            # Current pose as quaternion
            current_quat = Rotation.from_quat(self.current_pose[3:])
            
            # Convert delta_x rotation vector to quaternion
            delta_rot = Rotation.from_rotvec(delta_x[3:])
            
            # Combine rotations
            new_quat = current_quat * delta_rot
            
            # Create pose command
            cmd = Pose()
            cmd.position.x = self.current_pose[0] + delta_x[0]
            cmd.position.y = self.current_pose[1] + delta_x[1]
            cmd.position.z = self.current_pose[2] + delta_x[2]
            
            q = new_quat.as_quat()
            cmd.orientation.x = q[0]
            cmd.orientation.y = q[1]
            cmd.orientation.z = q[2]
            cmd.orientation.w = q[3]
            
            self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    
    controller = SimpleAdmittanceController()
    
    # Use multi-threaded executor for parallel processing
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        )
        
        # Create trajectory message
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = (self.current_joints + joint_velocities * (1.0/self.control_rate)).tolist()
        point.velocities = joint_velocities.tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int((1.0/self.control_rate) * 1e9)
        
        traj_msg.points.append(point)
        self.trajectory_pub.publish(traj_msg)

def main():
    rclpy.init()
    controller = SimpleAdmittanceController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
