#!/usr/bin/env python3
"""
Joint-level keyboard control for humanoid robot
Keys:
  Left arm (1-7): base_pitch, shoulder_yaw, shoulder_roll, elbow_pitch, wrist_pitch, wrist_yaw, gripper
  Right arm (q-u): base_pitch, shoulder_yaw, shoulder_roll, elbow_pitch, wrist_pitch, wrist_yaw, gripper
  Neck (a,s): yaw, pitch
  ESC: quit
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float64MultiArray, Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState

# ── Joint name configuration ──────────────────────────────────────────────────
# Modify these lists to match the joint names published by your hardware_interface.
# Order must correspond to the index used in self.left_joints / self.right_joints / self.neck_joints.
LEFT_JOINT_NAMES = [
    'left_base_pitch_joint',      
    'left_shoulder_roll_joint',   
    'left_shoulder_yaw_joint',    
    'left_elbow_pitch_joint',     
    'left_wrist_pitch_joint',     
    'left_wrist_yaw_joint',       
]

RIGHT_JOINT_NAMES = [
    'right_base_pitch_joint',     
    'right_shoulder_roll_joint',   
    'right_shoulder_yaw_joint',   
    'right_elbow_pitch_joint',    
    'right_wrist_pitch_joint',   
    'right_wrist_yaw_joint',     
]

NECK_JOINT_NAMES = [
    'neck_pitch_joint',   
    'neck_yaw_joint',     
]

# Timeout (seconds) to wait for the first /joint_states message
INIT_TIMEOUT_SEC = 5.0
# ──────────────────────────────────────────────────────────────────────────────

class JointKeyboardControl(Node):
    def __init__(self):
        super().__init__('joint_keyboard_control')
        
        # Publishers
        self.left_joint_pub = self.create_publisher(Float64MultiArray, '/left_joint_command', 10)
        self.right_joint_pub = self.create_publisher(Float64MultiArray, '/right_joint_command', 10)
        self.neck_joint_pub = self.create_publisher(Float64MultiArray, '/neck_joint_command', 10)
        self.left_gripper_pub = self.create_publisher(Bool, '/open_left_gripper', 10)
        self.right_gripper_pub = self.create_publisher(Bool, '/open_right_gripper', 10)
        
        # Joint states (6 DOF for arms, 2 DOF for neck)
        self.left_joints = [0.0] * 6
        self.right_joints = [0.0] * 6
        self.neck_joints = [0.0, 0.0]
        
        # Control parameters
        self.joint_step = 0.02  # radians
        
        # Read hardware_interface state_interface values via /joint_states
        self._read_initial_joint_states()

        self.get_logger().info('Joint Keyboard Control Started')
        self.print_instructions()
        
     # ── Initial state reader ───────────────────────────────────────────────────

    def _read_initial_joint_states(self):
        """
        Subscribe to /joint_states (published by ros2_control's JointStatesBroadcaster,
        which directly mirrors the hardware_interface state_interface values) and
        use the first received message to initialise left_joints, right_joints and
        neck_joints.

        Falls back to zero-initialisation when no message arrives within INIT_TIMEOUT_SEC.
        """
        self.get_logger().info(
            f'Waiting up to {INIT_TIMEOUT_SEC}s for /joint_states '
            f'(hardware_interface state_interface) …'
        )

        # Use BEST_EFFORT so we can receive messages even from a transient-local publisher
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._init_received = False

        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_init_callback,
            qos,
        )

        # Spin until we get a message or time out
        start = self.get_clock().now()
        while not self._init_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
            if elapsed >= INIT_TIMEOUT_SEC:
                self.get_logger().warn(
                    'Timed out waiting for /joint_states. '
                    'Initialising all joints to 0.0 rad.'
                )
                break

        # Subscription is no longer needed
        self.destroy_subscription(self._joint_state_sub)

    def _joint_states_init_callback(self, msg: JointState):
        """
        Parse a JointState message and populate the three joint arrays.

        The JointStatesBroadcaster publishes *all* joints in a single message;
        we pick out only the names we care about and map them to the correct index.
        """
        if self._init_received:
            return  # Only need the first message

        name_to_pos = dict(zip(msg.name, msg.position))

        # Helper: fill a joint list from a name list, warn if a name is missing
        def fill(joint_list, name_list):
            for idx, joint_name in enumerate(name_list):
                if joint_name in name_to_pos:
                    joint_list[idx] = name_to_pos[joint_name]
                    self.get_logger().info(
                        f'  [{joint_name}] init → {name_to_pos[joint_name]:.4f} rad'
                    )
                else:
                    self.get_logger().warn(
                        f'  [{joint_name}] not found in /joint_states, keeping 0.0 rad'
                    )

        fill(self.left_joints,  LEFT_JOINT_NAMES)
        fill(self.right_joints, RIGHT_JOINT_NAMES)
        fill(self.neck_joints,  NECK_JOINT_NAMES)

        self._init_received = True
        self.get_logger().info('Initial joint states loaded from hardware_interface.')

    # ── UI ────────────────────────────────────────────────────────────────────

    def print_instructions(self):
        print("\n" + "="*60)
        print("JOINT KEYBOARD CONTROL")
        print("="*60)
        print("Left Arm (1-6):  1=base_pitch 2=shoulder_roll 3=shoulder_yaw")
        print("                 4=elbow_pitch 5=wrist_pitch 6=wrist_yaw")
        print("Left Gripper:    7=toggle open/close")
        print("")
        print("Right Arm (q-y): q=base_pitch w=shoulder_roll e=shoulder_yaw")
        print("                 r=elbow_pitch t=wrist_pitch y=wrist_yaw")
        print("Right Gripper:   u=toggle open/close")
        print("")
        print("Neck (a,s):      a=yaw s=pitch")
        print("")
        print("Modifiers:       SHIFT=decrease, normal=increase")
        print("ESC or Ctrl+C:   Quit")
        print("="*60 + "\n")
    
    def get_key(self):
        """Get single keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
            # Handle escape sequences
            if key == '\x1b':
                key += sys.stdin.read(2)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def handle_key(self, key):
        """Process keypress and publish commands"""
        # Left arm joints (1-6)
        left_keys = {'1': 0, '2': 1, '3': 2, '4': 3, '5': 4, '6': 5,
                     '!': 0, '@': 1, '#': 2, '$': 3, '%': 4, '^': 5}
        # Right arm joints (q-y)
        right_keys = {'q': 0, 'w': 1, 'e': 2, 'r': 3, 't': 4, 'y': 5,
                      'Q': 0, 'W': 1, 'E': 2, 'R': 3, 'T': 4, 'Y': 5}
        # Neck joints (a=yaw[1], s=pitch[0]) — controller order: [pitch, yaw]
        neck_keys = {'a': 1, 's': 0, 'A': 1, 'S': 0}
        
        # Determine direction (uppercase/shift = decrease)
        direction = -1 if key.isupper() or key in '!@#$%^' else 1
        
        # Left arm control
        if key.lower() in left_keys:
            joint_idx = left_keys[key.lower() if key.islower() else key]
            self.left_joints[joint_idx] += direction * self.joint_step
            msg = Float64MultiArray()
            msg.data = self.left_joints
            self.left_joint_pub.publish(msg)
            self.get_logger().info(f'Left joint {joint_idx}: {self.left_joints[joint_idx]:.2f}')
            
        # Right arm control
        elif key.lower() in right_keys:
            joint_idx = right_keys[key]
            self.right_joints[joint_idx] += direction * self.joint_step
            msg = Float64MultiArray()
            msg.data = self.right_joints
            self.right_joint_pub.publish(msg)
            self.get_logger().info(f'Right joint {joint_idx}: {self.right_joints[joint_idx]:.2f}')
            
        # Neck control
        elif key.lower() in neck_keys:
            joint_idx = neck_keys[key]
            self.neck_joints[joint_idx] += direction * self.joint_step
            msg = Float64MultiArray()
            msg.data = self.neck_joints
            self.neck_joint_pub.publish(msg)
            self.get_logger().info(f'Neck joint {joint_idx}: {self.neck_joints[joint_idx]:.2f}')
            
        # Gripper control
        elif key == '7':
            msg = Bool()
            msg.data = True
            self.left_gripper_pub.publish(msg)
            self.get_logger().info('Left gripper: OPEN')
        elif key == '&':
            msg = Bool()
            msg.data = False
            self.left_gripper_pub.publish(msg)
            self.get_logger().info('Left gripper: CLOSE')
        elif key == 'u':
            msg = Bool()
            msg.data = True
            self.right_gripper_pub.publish(msg)
            self.get_logger().info('Right gripper: OPEN')
        elif key == 'U':
            msg = Bool()
            msg.data = False
            self.right_gripper_pub.publish(msg)
            self.get_logger().info('Right gripper: CLOSE')
            
        # Quit
        elif key == '\x1b' or key == '\x03':  # ESC or Ctrl+C
            return False
            
        return True
    
    def run(self):
        """Main control loop"""
        try:
            while rclpy.ok():
                key = self.get_key()
                if not self.handle_key(key):
                    break
        except KeyboardInterrupt:
            pass
        finally:
            self.get_logger().info('Shutting down...')


def main(args=None):
    rclpy.init(args=args)
    node = JointKeyboardControl()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
