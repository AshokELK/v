#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from gazebo_msgs.srv import ApplyBodyWrench
from builtin_interfaces.msg import Duration
import time

class ForceApplier(Node):
    def __init__(self):
        super().__init__('force_applier')
        
        # Create a client for the apply_body_wrench service
        self.client = self.create_client(ApplyBodyWrench, '/apply_body_wrench')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
    
    def apply_force(self, force_x=0.0, force_y=0.0, force_z=0.0,
                   torque_x=0.0, torque_y=0.0, torque_z=0.0,
                   duration=1.0):
        # Create the wrench message
        wrench = Wrench()
        wrench.force.x = force_x
        wrench.force.y = force_y
        wrench.force.z = force_z
        wrench.torque.x = torque_x
        wrench.torque.y = torque_y
        wrench.torque.z = torque_z
        
        # Create the request
        request = ApplyBodyWrench.Request()
        request.body_name = "ur::ft_sensor"  # The link to apply force to
        request.reference_frame = "world"    # Reference frame for the force
        request.reference_point.x = 0.0      # Point of force application
        request.reference_point.y = 0.0
        request.reference_point.z = 0.0
        request.wrench = wrench
        request.start_time.sec = 0           # Start immediately
        request.start_time.nanosec = 0
        request.duration = Duration(sec=int(duration))  # Duration to apply force
        
        # Send the request
        self.future = self.client.call_async(request)
        return self.future

def main():
    rclpy.init()
    force_applier = ForceApplier()
    
    try:
        # Example: Apply 10N force in X direction for 2 seconds
        future = force_applier.apply_force(force_x=10.0, duration=2.0)
        rclpy.spin_until_future_complete(force_applier, future)
        
        # Wait for the force duration
        time.sleep(2.0)
        
        # Example: Apply 5N force in Z direction for 1 second
        future = force_applier.apply_force(force_z=5.0, duration=1.0)
        rclpy.spin_until_future_complete(force_applier, future)
        
        # Wait for the force duration
        time.sleep(1.0)
        
        # Example: Apply torque around Y axis
        future = force_applier.apply_force(torque_y=2.0, duration=1.0)
        rclpy.spin_until_future_complete(force_applier, future)
        
    finally:
        force_applier.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
