# import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy, Imu
import numpy as np
from KaquIK.InverseKinematics import InverseKinematics  # IK 클래스
from Kaquctrl.RobotController import RobotController
from KaquCmdManager.CmdManagerNode import RobotState

class QuadrupedControllerNode(Node):
    def __init__(self):
        super().__init__('quadruped_controller_node')

        # 로봇 기하학 설정, params에서 가져와도됨
        self.body = [0., 0.]  # bodyLength, bodyWidth
        self.legs = [0., 0., 0., 0.]  # l1, l2, l3, l4

        # Gait Controller와 Inverse Kinematics 초기화
        # default_stance = np.array([
        #     [0.1, 0.1, -0.1, -0.1],
        #     [0.05, -0.05, 0.05, -0.05],
        #     [0.0, 0.0, 0.0, 0.0]
        # ])
        self.inverse_kinematics = InverseKinematics(self.body, self.legs)
        self.kaquctrl = RobotController(self.body, self.legs)

        # 퍼블리셔 설정
        self.command_topics = [
            "KAQU_FR1_joint/command",
            "KAQU_FR2_joint/command",
            "KAQU_FR3_joint/command",
            "KAQU_FL1_joint/command",
            "KAQU_FL2_joint/command",
            "KAQU_FL3_joint/command",
            "KAQU_RR1_joint/command",
            "KAQU_RR2_joint/command",
            "KAQU_RR3_joint/command",
            "KAQU_RL1_joint/command",
            "KAQU_RL2_joint/command",
            "KAQU_RL3_joint/command"
        ]

        self.data_type = Float64
        self.queue_size = 10

        self.publishers = self.joint_pub()
    

    def joint_pub(self):
        publishers = []
        for topic in self.command_topics:
            publisher = self.create_publisher(
                self.data_type,
                topic,
                self.queue_size
            )
            publishers.append(publisher)
        return publishers 
            
    def main_control(self):
        leg_position = self.kaquctrl.run()
        self.kaquctrl.select_gait()

        dx = RobotState.body_local_position[0]
        dy = RobotState.body_local_position[1]
        dz = RobotState.body_local_position[2]

        roll = RobotState.body_local_orientation[0]
        pitch = RobotState.body_local_orientation[1]
        yaw = RobotState.body_local_orientation[2]

        try:
            pub_angles = InverseKinematics.inverse_kinematics(leg_position, dx, dy, dz, roll, pitch, yaw)
    
            for i, angle in enumerate(pub_angles):
                msg = Float64()
                msg.data = angle
                self.publishers[i].publish(msg)
        except :   
            pass

def main(args=None):
    rclpy.init(args=args)
    node = QuadrupedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
