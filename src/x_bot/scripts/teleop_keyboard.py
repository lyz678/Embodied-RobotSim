#!/usr/bin/env python3
"""
Keyboard Teleop Control Node for X Bot.
Controls base movement, arm joints, and gripper using keyboard.

Key Bindings:
  Base Control:
    W/S - Forward/Backward
    A/D - Turn Left/Right
    
  Arm Joint Control:
    1-7 - Select joint (current shown below)
    +/= - Increase selected joint angle
    -/_ - Decrease selected joint angle
    
  Gripper Control:
    O - Open gripper
    P - Close gripper
    
  Other:
    Space - Stop all movement
    H - Home position (reset arm)
    Esc/Q - Quit
"""

import sys
import select
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand, FollowJointTrajectory
from builtin_interfaces.msg import Duration
import os
from ament_index_python.packages import get_package_share_directory
import yaml
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math

# Preset Poses (Position: [x, y, z], Orientation: [x, y, z, w])
PRESET_POSES = {
    'i': {'pos': [0.15, 0.00, 0.94], 'rot': [-0.00, 0.21, 0.01, 0.98], 'name': 'Init'},
    'k': {'pos': [-0.01, 0.48, 0.51], 'rot': [-0.50, 0.48, 0.52, 0.50], 'name': 'Pick Floor'},
    'l': {'pos': [-0.02, -0.76, 0.61], 'rot': [0.29, 0.28, -0.66, 0.64], 'name': 'Place'}
}


# Key bindings help message
HELP_MSG = """
╔══════════════════════════════════════════════════════════╗
║           X Bot Keyboard Teleop Control                ║
╠══════════════════════════════════════════════════════════╣
║  Base Control:                                           ║
║    W/S     - Forward/Backward                            ║
║    A/D     - Turn Left/Right                             ║
║    Z/X     - Decrease/Increase linear speed              ║
║    C/V     - Decrease/Increase angular speed             ║
║                                                          ║
║  Arm Joint Control:                                      ║
║    1-7     - Select joint (current shown below)          ║
║    +/=     - Increase joint angle (+0.1 rad)             ║
║    -/_     - Decrease joint angle (-0.1 rad)             ║
║    [/]     - Decrease/Increase joint step size           ║
║                                                          ║
║  Gripper Control:                                        ║
║    O       - Open gripper                                ║
║    P       - Close gripper                               ║
║                                                          ║
║  Other:                                                  ║
║    H       - Home position (reset arm)                   ║
║    Space   - Stop base movement                          ║
║    Space   - Stop base movement                          ║
║    I       - Move to Init Pose                           ║
║    K       - Move to Pick Floor Pose                     ║
║    L       - Move to Place Pose                          ║
║    Q/Esc   - Quit                                        ║
╚══════════════════════════════════════════════════════════╝
"""


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Base velocity publisher (use /x_bot/cmd_vel for Gazebo diff_drive)
        self.cmd_vel_pub = self.create_publisher(Twist, '/x_bot/cmd_vel', 10)
        
        # Pose publisher for Robot Actions
        self.pose_pub = self.create_publisher(PoseStamped, '/arm_command/pose', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Arm trajectory action client
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory')
        
        # Gripper action client
        self.gripper_action_client = ActionClient(
            self, GripperCommand, '/fr3_gripper_controller/gripper_cmd')
        
        # Arm joint names
        self.arm_joint_names = [
            'fr3_joint1',
            'fr3_joint2',
            'fr3_joint3',
            'fr3_joint4',
            'fr3_joint5',
            'fr3_joint6',
            'fr3_joint7',
        ]
        
        # Current joint positions (start at home)
        self.joint_positions = self.load_home_config()
        
        # Currently selected joint (0-6)
        self.selected_joint = 0
        
        # Base velocity parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        
        # Joint step size
        self.joint_step = 0.1  # rad
        
        # Gripper positions
        self.gripper_open = 0.04
        self.gripper_close = 0.005 # Updated based on user feedback to 0.005
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Load joint limits
        self.joint_limits = self.load_joint_limits()

        self.get_logger().info('Teleop Keyboard Node Started')
        print(HELP_MSG)
        self.print_status()
    
    def load_joint_limits(self):
        """Load joint limits from config"""
        limits = [] # List of (min, max) tuples
        config_path = os.path.join(get_package_share_directory('x_bot'), 'config', 'joint_limits.yaml')
        
        # Try finding in package if not absolute (not needed for this specific request but good practice)
        # Assuming absolute path provided in request is reliable or fallback to package
        if not os.path.exists(config_path):
             try:
                config_path = os.path.join(get_package_share_directory('x_bot'), 'config', 'joint_limits.yaml')
             except: pass

        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)
                
                # Assuming standard structure: joint1..7
                for i in range(1, 8):
                    key = f'joint{i}'
                    if key in config and 'limit' in config[key]:
                        lower = float(config[key]['limit']['lower'])
                        upper = float(config[key]['limit']['upper'])
                        limits.append((lower, upper))
                        self.get_logger().info(f"Joint {i} limits: [{lower}, {upper}]")
                    else:
                        limits.append((-3.14, 3.14)) # Default backup
                        self.get_logger().warn(f"No limit found for {key}, using default")
            except Exception as e:
                self.get_logger().warn(f"Failed to load limits: {e}")
                # Fallback
                return [(-3.14, 3.14)] * 7
        else:
             self.get_logger().warn("Limit config not found")
             return [(-3.14, 3.14)] * 7
        
        return limits
    
    def load_home_config(self):
        """Load home positions from configuration file"""
        joints = [0.0] * 7
        # Priority: User specified path -> Package share path
        paths = [
            os.path.join(get_package_share_directory('x_bot'), 'config', 'initial_positions.yaml'),
            os.path.join(get_package_share_directory('x_bot'), 'config', 'initial_positions.yaml')
        ]
        
        for path in paths:
            if os.path.exists(path):
                try:
                    with open(path, 'r') as f:
                        config = yaml.safe_load(f)
                    pos = config.get('initial_positions', {})
                    if pos:
                        for i in range(7):
                            joints[i] = float(pos.get(f'fr3_joint{i+1}', 0.0))
                        self.get_logger().info(f"Loaded home positions from {path}")
                        return joints
                except Exception as e:
                    self.get_logger().warn(f"Failed to read {path}: {e}")
        
        self.get_logger().warn("Could not load home positions, defaulting to zeros")
        return joints
    
    def print_status(self):
        """Print current status"""
        joint_names_short = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J7']
        # Protect against out of bounds index
        idx = self.selected_joint
        if 0 <= idx < len(joint_names_short):
            # Get Pose
            pose_str = "Pose: N/A"
            try:
                if self.tf_buffer.can_transform('odom', 'fingers_center', rclpy.time.Time()):
                    t = self.tf_buffer.lookup_transform('odom', 'fingers_center', rclpy.time.Time())
                    p = t.transform.translation
                    r = t.transform.rotation
                    pose_str = f"Pos:({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) Rot(xyzw):({r.x:.2f}, {r.y:.2f}, {r.z:.2f}, {r.w:.2f})"
            except:
                pass

            msg = f"\rJ:{joint_names_short[idx]} | Lin:{self.linear_speed:.2f} | Ang:{self.angular_speed:.2f} | Step:{self.joint_step:.2f} | {pose_str}    "
            print(msg, end='', flush=True)
    
    def get_key(self):
        """Get a single keypress"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            # Handle escape sequences (arrow keys, etc.)
            if key == '\x1b':
                key += sys.stdin.read(2)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def send_base_velocity(self, linear: float, angular: float):
        """Send velocity command to base"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
    
    def send_arm_trajectory(self):
        """Send arm joint trajectory"""
        if not self.arm_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Arm action server not available')
            return
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions.copy()
        point.time_from_start = Duration(sec=1, nanosec=0)
        goal.trajectory.points.append(point)
        
        self.arm_action_client.send_goal_async(goal)
    
    def send_gripper_command(self, position: float):
        """Send gripper command"""
        # Prevent spamming identical commands
        if hasattr(self, 'last_gripper_target') and self.last_gripper_target == position:
            return False
            
        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Gripper action server not available')
            return False
        
        self.last_gripper_target = position
        
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 10.0
        
        self.gripper_action_client.send_goal_async(goal)
        return True
    
    def home_arm(self):
        """Move arm to home position"""
        self.joint_positions = self.load_home_config()
    def home_arm(self):
        """Move arm to home position"""
        self.joint_positions = self.load_home_config()
        self.send_arm_trajectory()
        print(f"\n[INFO] Arm moving to home position")
        self.print_status()

    def send_preset_pose(self, pos, rot, name):
        """Send a preset Cartesian pose"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        
        # Normalize Quaternion to ensure validity
        norm = math.sqrt(rot[0]**2 + rot[1]**2 + rot[2]**2 + rot[3]**2)
        if norm > 0:
            rot = [r / norm for r in rot]
        
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        
        msg.pose.orientation.x = float(rot[0])
        msg.pose.orientation.y = float(rot[1])
        msg.pose.orientation.z = float(rot[2])
        msg.pose.orientation.w = float(rot[3])
        
        self.pose_pub.publish(msg)
        print(f"\n[INFO] Moving to {name} Pose: Pos({pos[0]}, {pos[1]}, {pos[2]}) Rot(xyzw)({rot[0]:.2f}, {rot[1]:.2f}, {rot[2]:.2f}, {rot[3]:.2f})")
        self.print_status()
            

    
    def run(self):
        """Main loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '':
                    continue
                
                # Quit
                if key in ['q', 'Q', '\x1b', '\x03']:  # q, Q, Esc, Ctrl+C
                    print("\n[INFO] Exiting teleop...")
                    break
                
                # Base control
                elif key.lower() == 'w':
                    self.send_base_velocity(self.linear_speed, 0.0)
                elif key.lower() == 's':
                    self.send_base_velocity(-self.linear_speed, 0.0)
                elif key.lower() == 'a':
                    self.send_base_velocity(0.0, self.angular_speed)
                elif key.lower() == 'd':
                    self.send_base_velocity(0.0, -self.angular_speed)
                elif key == ' ':
                    self.send_base_velocity(0.0, 0.0)
                    print("\n[INFO] Base stopped")
                    self.print_status()
                
                # Joint selection (1-7)
                elif key in '1234567':
                    self.selected_joint = int(key) - 1
                    self.print_status()
                
                # Joint control (+/-)
                elif key in ['+', '=']:
                    # Update joint with new value
                    new_val = self.joint_positions[self.selected_joint] + self.joint_step
                    # Clamp
                    mn, mx = self.joint_limits[self.selected_joint]
                    self.joint_positions[self.selected_joint] = max(mn, min(mx, new_val))
                    
                    self.send_arm_trajectory()
                    self.print_status()
                elif key in ['-', '_']:
                    new_val = self.joint_positions[self.selected_joint] - self.joint_step
                    # Clamp
                    mn, mx = self.joint_limits[self.selected_joint]
                    self.joint_positions[self.selected_joint] = max(mn, min(mx, new_val))
                    
                    self.send_arm_trajectory()
                    self.print_status()
                
                # Gripper control
                elif key.lower() == 'o':
                    if self.send_gripper_command(self.gripper_open):
                        print("\n[INFO] Opening gripper")
                        self.print_status()
                elif key.lower() == 'p':
                    if self.send_gripper_command(self.gripper_close):
                        print("\n[INFO] Closing gripper")
                        self.print_status()
                
                # Home position
                elif key.lower() == 'h':
                    self.home_arm()
                
                # Preset Poses
                elif key.lower() in PRESET_POSES:
                    pose_data = PRESET_POSES[key.lower()]
                    self.send_preset_pose(
                        pose_data['pos'],
                        pose_data['rot'],
                        pose_data['name']
                    )
                
                # Speed control
                elif key.lower() == 'z':  # Decrease linear speed
                    self.linear_speed = max(0.05, self.linear_speed - 0.05)
                    print(f"\n[INFO] Linear speed: {self.linear_speed:.2f} m/s")
                    self.print_status()
                elif key.lower() == 'x':  # Increase linear speed
                    self.linear_speed = min(1.0, self.linear_speed + 0.05)
                    print(f"\n[INFO] Linear speed: {self.linear_speed:.2f} m/s")
                    self.print_status()
                elif key.lower() == 'c':  # Decrease angular speed
                    self.angular_speed = max(0.1, self.angular_speed - 0.1)
                    print(f"\n[INFO] Angular speed: {self.angular_speed:.2f} rad/s")
                    self.print_status()
                elif key.lower() == 'v':  # Increase angular speed
                    self.angular_speed = min(2.0, self.angular_speed + 0.1)
                    print(f"\n[INFO] Angular speed: {self.angular_speed:.2f} rad/s")
                    self.print_status()
                
                # Joint step size control
                elif key == '[':  # Decrease joint step
                    self.joint_step = max(0.01, self.joint_step - 0.05)
                    print(f"\n[INFO] Joint step: {self.joint_step:.2f} rad")
                    self.print_status()
                elif key == ']':  # Increase joint step
                    self.joint_step = min(0.5, self.joint_step + 0.05)
                    print(f"\n[INFO] Joint step: {self.joint_step:.2f} rad")
                    self.print_status()
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Stop base on exit
            self.send_base_velocity(0.0, 0.0)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)
    
    node = TeleopKeyboard()
    
    # Spin in a separate thread to handle ROS callbacks
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop everything safely
        if rclpy.ok():
            rclpy.shutdown()
        
        try:
            node.destroy_node()
        except:
            pass


if __name__ == '__main__':
    main()
