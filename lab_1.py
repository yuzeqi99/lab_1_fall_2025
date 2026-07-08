import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
from collections import deque
import signal

JOINT_NAME = "leg_front_l_1"
####
####
KP = 2.0  # YOUR KP VALUE
KD = 0.0  # YOUR KD VALUE
####
####
LOOP_RATE = 200  # Hz
MAX_TORQUE = 3.0


class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__("joint_state_subscriber")
        # Create a subscriber to the /joint_states topic
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self.get_joint_info, 10  # QoS profile history depth
        )
        self.subscription  # prevent unused variable warning

        # Publisher to the /forward_command_controller/commands topic
        self.command_publisher = self.create_publisher(Float64MultiArray, "/forward_command_controller/commands", 10)
        self.print_counter = 0
        self.calculated_torque = 0
        self.joint_pos = 0
        self.joint_vel = 0
        self.target_joint_pos = 0
        self.target_joint_vel = 0
        # self.torque_history = deque(maxlen=DELAY)

        # Create a timer to run control_loop at the specified frequency
        self.create_timer(1.0 / LOOP_RATE, self.control_loop) #loop_Rate = 200hz,这个函数的作用相当于每秒调用control_loop 200次

    def get_target_joint_info(self):
        ####
        #### YOUR CODE HERE
        ####
        target_joint_pos = 0.5 #设置一个固定值用于测试
        target_joint_vel = 0.0 #机器人关节到达目标位置时必须停下，保持这个角度不动，所以期望速度必须为0

        # target_joint_pos, target_joint_vel
        return target_joint_pos, target_joint_vel

    def calculate_torque(self, joint_pos, joint_vel, target_joint_pos, target_joint_vel):
        ####
        #### YOUR CODE HERE
        ####
        error = target_joint_pos - joint_pos

        torque = KP * error #若误差越大给的力矩越大，误差越小给的力矩越小，更加平滑
        
        return torque

    def print_info(self):
        """Print joint information every 2 control loops"""
        if self.print_counter == 0: #当计数器=0时，打印日志 
            self.get_logger().info(
                f"Pos: {self.joint_pos:.2f}, Target Pos: {self.target_joint_pos:.2f}, Vel: {self.joint_vel:.2f}, Target Vel: {self.target_joint_vel:.2f}, Tor: {self.calculated_torque:.2f}"
            )
        #隔一轮打印一次
        self.print_counter += 1
        self.print_counter %= 2

    def get_joint_info(self, msg): #获取机器人左前腿关节1的角度和角速度
        """Callback function to process incoming JointState messages"""
        joint_index = msg.name.index(JOINT_NAME)
        joint_pos = msg.position[joint_index]
        joint_vel = msg.velocity[joint_index]

        #更新机器人左前腿关节1(髋关节)的状态信息
        self.joint_pos = joint_pos 
        self.joint_vel = joint_vel

        return joint_pos, joint_vel

    def control_loop(self):
        """Control control loop to calculate and publish torque commands"""
        #更新目标关节角度和角速度
        self.target_joint_pos, self.target_joint_vel = self.get_target_joint_info() 
        #更新力矩
        self.calculated_torque = self.calculate_torque(  
            self.joint_pos, self.joint_vel, self.target_joint_pos, self.target_joint_vel
        )
        self.print_info()
        #发送力矩数据
        self.publish_torque(self.calculated_torque)

    def publish_torque(self, torque=0.0):
        # Create a Float64MultiArray message with zero kp and kd values
        command_msg = Float64MultiArray()
        #限制力矩大小，防止超过机器限制
        torque = np.clip(torque, -MAX_TORQUE, MAX_TORQUE)
        command_msg.data = [torque, 0.0, 0.0]  # Zero kp and kd values

        # Publish the message
        self.command_publisher.publish(command_msg)


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    joint_state_subscriber = JointStateSubscriber() #建立实力，命名为joint_state_subscriber

    # Install a SIGINT handler that sends zero torque BEFORE shutting down the context 安全降落伞
    def _handle_sigint(sig, frame): 
        joint_state_subscriber.get_logger().info("SIGINT received: sending zero torque and shutting down...") #提示收到中断信号
        joint_state_subscriber.publish_torque(0.0)
        time.sleep(0.1)  #等待0.1s
        joint_state_subscriber.publish_torque(0.0) #Ros处理消息有延迟，发送两次确保”零力矩“被彻底接收并执行
        rclpy.shutdown() #安全关闭

    signal.signal(signal.SIGINT, _handle_sigint)

    rclpy.spin(joint_state_subscriber) #阻塞式循环，会不断检查是否有新的/joint_states消息，如果有就触发get_joint_info函数
  

if __name__ == "__main__":
    main()
