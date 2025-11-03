import math

class Kinematics:
    # L1 (arm 1 length), L2 (arm 2 length)
    def __init__(self, L1, L2):
        self.L1 = L1
        self.L2 = L2

    # Inputs: x (forward/backward), y (height)
    def inverse_kinematics(self, x, y):
        x = -x
        
        # Law of cosines
        cos_angle2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        angle2 = math.acos(max(min(cos_angle2, 1), -1))
        
        # Law of sines
        k1 = self.L1 + self.L2 * math.cos(angle2)
        k2 = self.L2 * math.sin(angle2)
        angle1 = math.atan2(y, x) - math.atan2(k2, k1)
        
        return math.degrees(angle1), math.degrees(angle2)
    
    def forward_kinematics(self, angle1_deg, angle2_deg):
        """
        Calculate the end-effector position (x, y) from joint angles.
        
        Args:
            angle1_deg: Angle of first joint in degrees (from negative x-axis)
            angle2_deg: Angle of second joint in degrees (relative to first link)
        
        Returns:
            Tuple (x, y) representing the end-effector position
        """
        angle1 = math.radians(angle1_deg)
        angle2 = math.radians(angle2_deg)
        x1 = self.L1 * math.cos(angle1)
        y1 = self.L1 * math.sin(angle1)
        
        absolute_angle2 = angle1 + angle2
        x2 = x1 + self.L2 * math.cos(absolute_angle2)
        y2 = y1 + self.L2 * math.sin(absolute_angle2)
        
        return -x2, y2

# k = Kinematics(20,20)
# t1,t2=k.forward_kinematics(100,101)
# print(t1)
# print(t2)
