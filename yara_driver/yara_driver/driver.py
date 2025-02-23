import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import serial
import pygame 
import struct
import time
import serial.tools.list_ports
from control_msgs.action import FollowJointTrajectory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer
import array

class RobotStatePublisher(Node):

    def __init__(self):
        super().__init__('yara_state_publisher')

        self.action_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = ReentrantCallbackGroup()

        self.timer = self.create_timer(0.01, self.timer_callback, callback_group=self.timer_callback_group)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.joint_state = JointState()

        self.joint_state.name = ['yara_joint_1', 'yara_joint_2', 'yara_joint_3', 'yara_joint_4', 'yara_joint_5']

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'yara_arm_controller/srki_ns',
            self.execute_callback,
            callback_group=self.action_callback_group
        )

        self.fake_yara_angles = [0, 0, 0, 0, 0]
        self.yara_angles = [0, 0, 0, 0, 0]

        ports = serial.tools.list_ports.comports()
        self.ser = None
        self.msg = bytearray()
        self.joint = 0
        self.msg_done = False
        self.executing_trajectory = False

        self.time = 0

        self.declare_parameter(name="joystick")
        self.declare_parameter(name="hardware")

        self.use_joystick = self.get_parameter('joystick').get_parameter_value().bool_value
        self.use_hardware = self.get_parameter('hardware').get_parameter_value().bool_value

        if(self.use_joystick):
            pygame.joystick.init()
            pygame.init()
            self._joystick = pygame.joystick.Joystick(0)
            self._joystick.init()
            self.arm_control_timer = self.create_timer(0.01, self.control_timer_callback)
        if(self.use_hardware):
            for port, desc, _ in sorted(ports):
                    if "STM" in desc:
                        try:
                            self.ser = serial.Serial(str(port), 115200)
                            print(f'Successfully opened port: {desc}')
                        except Exception as e:
                            print(f'error trying to open port: {e}')
                        time.sleep(0.1)
        
            self.init_robot_arm()
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('===============Executing trajectory...===============')
        self.executing_trajectory = True
        feedback_msg = FollowJointTrajectory.Feedback()

        trajectory = goal_handle.request.trajectory
        final_joint_angles = trajectory.points[-1].positions
        if(self.use_hardware):
            self.send_command([g_i - c_i for g_i, c_i in zip(final_joint_angles, self.yara_angles)])

        for point in trajectory.points:
            feedback_msg.actual.positions = point.positions            
            feedback_msg.desired.positions = point.positions
            feedback_msg.error.positions = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.fake_yara_angles = point.positions
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.05)

        self.yara_angles = trajectory.points[-1].positions

        result = FollowJointTrajectory.Result()
        goal_handle.succeed()
        result.error_code = 0
        self.executing_trajectory = False
        return result

    def init_robot_arm(self):
        msg1 = bytearray(struct.pack("<f", 1))
        self.msg.append(0xAC)
        self.msg.append(0xAC)

        for b in msg1:
            self.msg.append(b)
        self.msg_done = True

        if (self.msg_done):
            self.ser.write(self.msg)
            print('Init done')
            self.msg = bytearray()
            self.msg_done = False


    def send_command(self, joint_angles):
        cmd_msg = bytearray()
        msg1 = bytearray(struct.pack("<f", math.degrees(joint_angles[0])))
        cmd_msg.append(0xAA)
        cmd_msg.append(0xA1)
        for b in msg1:
            cmd_msg.append(b)
        msg1 = bytearray(struct.pack("<f", math.degrees(joint_angles[1])))
        cmd_msg.append(0xAA)
        cmd_msg.append(0xA2)
        for b in msg1:
            cmd_msg.append(b)
        msg1 = bytearray(struct.pack("<f", math.degrees(joint_angles[2])))
        cmd_msg.append(0xAA)
        cmd_msg.append(0xA3)
        for b in msg1:
            cmd_msg.append(b)
        msg1 = bytearray(struct.pack("<f", math.degrees(joint_angles[3])))
        cmd_msg.append(0xAA)
        cmd_msg.append(0xA4)
        for b in msg1:
            cmd_msg.append(b)
        msg1 = bytearray(struct.pack("<f", math.degrees(joint_angles[4])))
        cmd_msg.append(0xAA)
        cmd_msg.append(0xA5)
        for b in msg1:
            cmd_msg.append(b)

        self.ser.write(cmd_msg)
        self.get_logger().info(f"Poslao uglove {"".join([hex(i) for i in cmd_msg])}")
        self.get_logger().info(f"Poslao uglove {joint_angles[0], math.degrees(joint_angles[1]), math.degrees(joint_angles[2]), math.degrees(joint_angles[3]), math.degrees(joint_angles[4])}")

    def control_timer_callback(self):
        axis1 = self._joystick.get_axis(0)
        axis2 = self._joystick.get_axis(1)
        axis3 = self._joystick.get_axis(4)
        axis4 = self._joystick.get_axis(3)
        axis5_cw = self._joystick.get_axis(2)
        axis5_ccw = self._joystick.get_axis(5)
        pygame.event.pump()

        if axis1 != 0:
            self.joint = 0xA1
            if(axis1 > 0.8):
                msg1 = bytearray(struct.pack("<f", 1))
                self.msg.append(0xAA)
                self.msg.append(self.joint)
                for b in msg1:
                    self.msg.append(b)
                self.msg_done = True
            elif(axis1 < -0.8):
                msg1 = bytearray(struct.pack("<f", -1))
                self.msg.append(0xAA)
                self.msg.append(self.joint)
                for b in msg1:
                    self.msg.append(b)
                self.msg_done = True
            if (self.msg_done):
                self.ser.write(self.msg)
                self.msg = bytearray()
                self.msg_done = False

        if axis2 != 0:
            self.joint = 0xA2
            if(axis2 > 0.5):
                msg1 = bytearray(struct.pack("<f", 1))
                self.msg.append(0xAA)
                self.msg.append(self.joint)
                for b in msg1:
                    self.msg.append(b)
                self.msg_done = True
            elif(axis2 < -0.5):
                msg1 = bytearray(struct.pack("<f", -1))
                self.msg.append(0xAA)
                self.msg.append(self.joint)
                for b in msg1:
                    self.msg.append(b)
                self.msg_done = True
            if (self.msg_done):
                self.ser.write(self.msg)
                self.msg = bytearray()
                self.msg_done = False

        if axis3 != 0:
            self.joint = 0xA3
            if(axis3 > 0.1):
                msg1 = bytearray(struct.pack("<f", 1))
                self.msg.append(0xAA)
                self.msg.append(self.joint)
                for b in msg1:
                    self.msg.append(b)
                self.msg_done = True
            elif(axis3 < -0.1):
                msg1 = bytearray(struct.pack("<f", -1))
                self.msg.append(0xAA)
                self.msg.append(self.joint)
                for b in msg1:
                    self.msg.append(b)
                self.msg_done = True
            if (self.msg_done):
                self.ser.write(self.msg)
                self.msg = bytearray()
                self.msg_done = False

        if axis4 != 0:
            self.joint = 0xA4
            if(axis4 > 0.8):
                msg1 = bytearray(struct.pack("<f", 1))
                self.msg.append(0xAA)
                self.msg.append(self.joint)
                for b in msg1:
                    self.msg.append(b)
                self.msg_done = True
            elif(axis4 < -0.8):
                msg1 = bytearray(struct.pack("<f", -1))
                self.msg.append(0xAA)
                self.msg.append(self.joint)
                for b in msg1:
                    self.msg.append(b)
                self.msg_done = True
            if (self.msg_done):
                self.ser.write(self.msg)
                self.msg = bytearray()
                self.msg_done = False

        if axis5_cw > 0 or axis5_ccw > 0:
            self.joint = 0xA5
            if(axis5_cw > 0):
                msg1 = bytearray(struct.pack("<f", 1))
                self.msg.append(0xAA)
                self.msg.append(self.joint)
                for b in msg1:
                    self.msg.append(b)
                self.msg_done = True
            elif(axis5_ccw > 0):
                msg1 = bytearray(struct.pack("<f", -1))
                self.msg.append(0xAA)
                self.msg.append(self.joint)
                for b in msg1:
                    self.msg.append(b)
                self.msg_done = True
            if (self.msg_done):
                self.ser.write(self.msg)
                self.msg = bytearray()
                self.msg_done = False
        
        if(self.use_joystick and not self.executing_trajectory):
            msg = bytearray()
            msg1 = bytearray(struct.pack("<f", 1))
            msg.append(0xAD)
            msg.append(0xAD)

            for b in msg1:
                msg.append(b)
            self.ser.write(msg)

            ret_msg = self.ser.read(20)
            arr = array.array('f', ret_msg)
            arr.tolist()
            arr.tolist()

            self.fake_yara_angles[0] = math.radians(arr[0])
            self.fake_yara_angles[1] = math.radians(arr[1])
            self.fake_yara_angles[2] = math.radians(arr[2])
            self.fake_yara_angles[3] = math.radians(arr[3])
            self.fake_yara_angles[4] = math.radians(arr[4])

    def timer_callback(self):        
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.position=self.fake_yara_angles
        self.joint_state_publisher.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)

    try:
        executor = rclpy.executors.MultiThreadedExecutor()
        minimal_publisher = RobotStatePublisher()
        rclpy.spin(node = minimal_publisher, executor=executor)
    except KeyboardInterrupt:
        minimal_publisher.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
