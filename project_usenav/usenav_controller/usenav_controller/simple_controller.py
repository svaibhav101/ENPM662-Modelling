#!/usr/bin/env python3
import time
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.j1_pub = self.create_publisher(Float64, '/joint1_cmd', 10)
        self.j2_pub = self.create_publisher(Float64, '/joint2_cmd', 10)
        self.j3_pub = self.create_publisher(Float64, '/joint3_cmd', 10)
        self.j4_pub = self.create_publisher(Float64, '/joint4_cmd', 10)
        self.j5_pub = self.create_publisher(Float64, '/joint5_cmd', 10)

        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0] 
        self.get_logger().info("Simple Controller Started")

    def get_dh_transform(self, theta, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d   ],
            [0,   0,      0,     1   ]
        ])

    def compute_fk_and_jacobian(self, q):
        dh_params = [
            [q[0],      0.2,  0.0,  np.pi/2], 
            [q[1],      0.0,  0.5,  0.0],     
            [q[2],      0.0,  0.4,  0.0],     
            [q[3],      0.0,  0.0, -np.pi/2], 
            [q[4],      0.05, 0.0,  0.0]      
        ]
        T_accum = np.eye(4)
        origins = [np.array([0,0,0])] 
        z_axes  = [np.array([0,0,1])]
        for params in dh_params:
            Ti = self.get_dh_transform(*params)
            T_accum = T_accum @ Ti
            origins.append(T_accum[:3, 3])
            z_axes.append(T_accum[:3, 2])

        end_pos = origins[-1]
        J = np.zeros((3, 5))
        for i in range(5):
            J[:, i] = np.cross(z_axes[i], end_pos - origins[i])
        return end_pos, J

    def solve_ik_numerical(self, target_x, target_y, target_z):
        q = np.array([0.0, 0.5, 1.0, -0.5, 0.0]) 
        target_pos = np.array([target_x, target_y, target_z])
        for i in range(50):
            curr, J = self.compute_fk_and_jacobian(q)
            err = target_pos - curr
            if np.linalg.norm(err) < 0.005: # 5mm tolerance
                return list(q)
            q += 0.5 * np.linalg.pinv(J) @ err
        return list(q)

    # --- EXPLICIT VALIDATION FUNCTION ---
    def validate_solution(self, q_sol, target):
        """
        Takes the IK solution (joint angles) and the Target (x,y,z).
        Computes FK to see where those angles actually put the robot.
        """
        actual_pos, _ = self.compute_fk_and_jacobian(q_sol)
        error = np.linalg.norm(np.array(target) - actual_pos)
        
        self.get_logger().info("="*40)
        self.get_logger().info("VALIDATION REPORT:")
        self.get_logger().info(f"  Target Coord: {target}")
        self.get_logger().info(f"  IK Solution : {[round(x,3) for x in q_sol]}")
        self.get_logger().info(f"  FK Result   : {[round(x,3) for x in actual_pos]}")
        self.get_logger().info(f"  TOTAL ERROR : {error:.5f} meters")
        self.get_logger().info("="*40)

    def publish_arm(self, q):
        self.j1_pub.publish(Float64(data=float(q[0])))
        self.j2_pub.publish(Float64(data=float(q[1])))
        self.j3_pub.publish(Float64(data=float(q[2])))
        self.j4_pub.publish(Float64(data=float(q[3])))
        self.j5_pub.publish(Float64(data=float(q[4])))

    def move_arm_smooth(self, target_q, duration=2.0):
        steps = int(duration * 50)
        start_q = np.array(self.current_joints)
        end_q = np.array(target_q)
        for i in range(steps):
            fraction = (i / steps)
            self.publish_arm(start_q + (end_q - start_q) * fraction)
            time.sleep(0.02)
        self.publish_arm(end_q)
        self.current_joints = list(end_q)
        time.sleep(0.5)

    def move_base(self, linear_x, angular_z, duration):
        msg = Twist()
        msg.linear.x = float(linear_x) 
        msg.angular.z = float(angular_z)
        end_time = time.time() + duration
        while time.time() < end_time:
            self.cmd_vel_pub.publish(msg)
            time.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())

    def run_sequence(self):
        time.sleep(2)
        
        # TARGET DEFINITION
        # Relative to robot base frame
        target_x, target_y, target_z = 0.6, 0.0, 0.1
        
        self.get_logger().info(">>> PHASE 1: WAREHOUSE NAVIGATION")
        self.move_base(0.5, 0.0, 1.5) 
        self.move_base(0.0, 0.5, 3.0) 
        time.sleep(0.5)
        self.move_base(0.0, -0.5, 3.0) 
        self.move_base(0.0, -0.5, 3.0)
        time.sleep(0.5)
        self.move_base(0.0, 0.5, 3.0)

        self.get_logger().info(">>> PHASE 2: APPROACHING TARGET SHELF")
        self.move_base(0.5, 0.0, 2.0) 
        
        self.get_logger().info(f">>> PHASE 3: SOLVING IK for ({target_x}, {target_y}, {target_z})")
        GRASP = self.solve_ik_numerical(target_x, target_y, target_z)
        
        # *** VALIDATION STEP ***
        self.validate_solution(GRASP, [target_x, target_y, target_z])
        
        APPROACH_POSE = [0.0, 0.0, 0.5, -0.5, 0.0]
        LIFT_POSE     = [0.0, -0.2, -0.5, 0.0, 0.0]
        SIDE_POSE     = [1.57, -0.2, -0.5, 0.0, 0.0]

        self.move_arm_smooth(APPROACH_POSE, 2.0)
        self.move_arm_smooth(GRASP, 1.5) 
        time.sleep(1.0) 
        
        self.move_arm_smooth(LIFT_POSE, 2.0)
        
        self.get_logger().info(">>> PHASE 4: SPATIAL DEMONSTRATION")
        self.move_arm_smooth(SIDE_POSE, 2.0)
        
        self.get_logger().info(">>> PHASE 5: RETURN TO DOCK")
        self.move_base(-0.5, 0.0, 3.5)
        self.move_arm_smooth([0.0]*5, 2.0)
        self.get_logger().info("Mission Complete")

    def run_sequence2(self):

        # TARGET DEFINITION
        # Relative to robot base frame
        target_x, target_y, target_z = 0.6, 0.0, 0.1
        
        self.get_logger().info(">>> PHASE 1: WAREHOUSE NAVIGATION")
        self.move_base(0.5, 0.0, 1.5) 
        self.move_base(0.0, 0.5, 3.0) 
        time.sleep(0.5)
        self.move_base(0.0, -0.5, 3.0) 
        self.move_base(0.0, -0.5, 3.0)
        time.sleep(0.5)
        self.move_base(0.0, 0.5, 3.0)

        self.get_logger().info(">>> PHASE 2: APPROACHING TARGET SHELF")
        self.move_base(0.5, 0.0, 2.0) 
        
        self.get_logger().info(f">>> PHASE 3: SOLVING IK for ({target_x}, {target_y}, {target_z})")
        GRASP = self.solve_ik_numerical(target_x, target_y, target_z)
        
        # *** VALIDATION STEP ***
        self.validate_solution(GRASP, [target_x, target_y, target_z])
        
        APPROACH_POSE = [0.0, 0.0, 0.5, -0.5, 0.0]
        LIFT_POSE     = [0.0, -0.2, -0.5, 0.0, 0.0]
        SIDE_POSE     = [1.57, -0.2, -0.5, 0.0, 0.0]

        self.move_arm_smooth(APPROACH_POSE, 2.0)
        self.move_arm_smooth(GRASP, 1.5) 
        time.sleep(1.0) 
        
        self.move_arm_smooth(LIFT_POSE, 2.0)
        
        self.get_logger().info(">>> PHASE 4: SPATIAL DEMONSTRATION")
        self.move_arm_smooth(SIDE_POSE, 2.0)
        
        self.get_logger().info(">>> PHASE 5: RETURN TO DOCK")
        self.move_base(-0.5, 0.0, 3.5)
        self.move_arm_smooth([0.0]*5, 2.0)
        self.get_logger().info("Mission Complete")

        

def main(args=None):
    rclpy.init(args=args)
    SimpleController().run_sequence()
    rclpy.shutdown()