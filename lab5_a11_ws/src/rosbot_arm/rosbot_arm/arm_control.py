import rclpy
from rclpy.node import Node
from ros_robot_controller.ros_robot_controller_sdk import Board
from rosbot_msgs.srv import CurrentAngle
from rosbot_msgs.msg import ForwardPose
from rosbot_msgs.msg import Gripper


class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control')
        self.forward_control_sub = self.create_subscription(ForwardPose, "forward_topic", self.forward_callback, 10)
        self.gripper_control_sub = self.create_subscription(Gripper, "gripper_topic", self.gripper_callback, 10)
        # TODO: Subscribe to a forward control topic
        # TODO: Subscribe to a gripper control topic
    
    board = Board()
    board.enable_reception()

    # TODO: Write an arm_control node to receive joint angles
    # and send messages to the Board.
    
    def forward_callback(self, msg):
        ids = []
        print("Message Recieved")
        for i in msg.ids:
            ids.append(i)

        for i in range(len(msg.pulses)):
                if msg.pulses[1] < 125:
                    msg.pulses[1] = 125

                elif msg.pulses[1] > 775:
                    msg.pulses[1] = 775

    

        targets = list(zip(ids, msg.pulses))

        self.board.bus_servo_set_position(msg.duration, targets)
        print("End")


    def gripper_callback(self, msg):
        id = 10
        pulse = msg.pulses
        if pulse > 650:
            pulse = 650
             
        target = list(zip(id, pulse))

        self.board.bus_servo_set_position(msg.duration, target)

            
        
        

    # See arm_test for example of board functionality.


def main():
    rclpy.init()
    node = ArmControl()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__=="__main__":
    main()
