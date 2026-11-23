import rclpy
from rclpy.node import Node
import math
import numpy as np

from rosbot_msgs.msg import ForwardPose
from rosbot_msgs.msg import Inverse

##  Joint Links ##
# l0 refers to the height of the arm base wrt base_link
l0 = 0.05 + 0.0654868 + 0.0338648 + 0.0772047
l1 = 0.130
l2 = 0.130
l3 = 0.055
tool_link = 0.117

lengths = [l0, l1, l2, l3+tool_link, 0]


arm_ids = [1, 2, 3, 4, 5]

# Only joint 2 is limited to protect the screen
min_pulses = [0, 125, 0, 0, 0]
max_pulses = [1000, 775, 1000, 1000, 1000]
min_angles = [-120, -90, -120, -120, -120]
max_angles = [120, 66, 120, 120, 120]



# All joints operate in the range -120 to 120 with a pulse width range of 0 to 1000

# TODO: Write helper scripts to convert from one to the other.

def angle_to_pulse(angle):
    # TODO
    return int(round(500 + angle * (1000 / 240)))

def pulse_to_angle(pulse):
    # TODO
    return (pulse - 500) * (240 / 1000)

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
    # TODO: Write inverse kinematics node
        self.forward_pub = self.create_publisher(ForwardPose, "forward_topic", 10)

        self.Inverse_sub = self.create_subscription(Inverse, "Inverse_topic", self.inverse_callback, 10)

    def inverse_callback(self, msg):
        print("IK_node called")

        if len(msg.xyz) != 3:
            self.get_logger().warn("xyz must contain 3 values.")
            return
        
        
        x, y, z = msg.xyz
        


        for sign in (1, -1):
            angles = self.SolveIK(x, y, z, msg.roll, msg.pitch, sign)
            if angles is None:
                continue
            
            angles_deg = [np.rad2deg(angle) for angle in angles]


            pulses = [int(angle_to_pulse(angle)) for angle in angles_deg]

            valid = True


            for i in range(5):
                if pulses[i] < min_pulses[i] or pulses[i] > max_pulses[i]:
                    valid = False
                    break
        
            if not valid:
                continue
            
            ##print("Pose Published", pulses)

            forward_msg = ForwardPose()
            forward_msg.ids = [1, 2, 3, 4, 5]
            forward_msg.pulses = pulses
            self.forward_pub.publish(forward_msg)
            print("Pose Published", pulses)
            return
    
        self.get_logger().warn(
            "Target is unreachable or exceeds joint limits."
        )
    
    def SolveIK(self,x,y, z, roll, pitch, elbow_sign):
        Angles = [0, 0, 0, 0, 0]

        roll = math.radians(roll)
        pitch = math.radians(pitch)

        # Reject invalid numeric inputs.
        if not all(
            math.isfinite(value)
            for value in (x, y, z, roll, pitch)
        ):
            return None
        
        th1 = math.atan2(y,x)
        r = math.hypot(x, y)

        if r < 1e-9:
            return None

        l4 = l3 + tool_link
        x4 = r - l4*math.cos(pitch)
        z4 = z - l4*math.sin(pitch) - l0

        D = math.sqrt(x4**2 + z4**2)
        if D > (l1+l2):
            return None

        L32 = math.sqrt(x4**2 + z4**2)

        cos_th3 = (L32**2 - l1**2 - l2**2) / (2 * l1 * l2)

        if cos_th3 < -1.0 - 1e-9 or cos_th3 > 1.0 + 1e-9:
            return None

        
        cos_th3 = max(-1.0, min(1.0, cos_th3))

        th3 = elbow_sign * math.acos(cos_th3)

        "beta = math.pi - th3"

        th2 = math.atan2(z4, x4) - math.atan2(l2*math.sin(th3), l1 + l2*math.cos(th3))

        th4 = pitch - th3 - th2
        
        th2 -= math.pi/2

        th5 = roll

        Angles[0] = th1
        Angles[1] = th2
        Angles[2] = th3
        Angles[3] = th4
        Angles[4] = th5

        print("Angles = ", Angles)

        return Angles

         


def main():
    rclpy.init()
    ik_node = IKNode()
    rclpy.spin(ik_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
