import rospy
import hsrb_interface
from sensor_msgs.msg import LaserScan
from hsrb_interface import Robot, geometry

robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
tts = robot.get("default_tts")

class Door():
    def __init__(self):
        print("Door Open")
        self.range = 0
        rospy.Subscriber("/hsrb/base_scan",LeaserScan,scan_callback)
 
    def scan_callback(self,msg):
       self.range = msg.ranges
    
    def main(self):
        if min(self.range) < 0.5:
            print("not open")
            tts.say("ドアが空いていません")
            rospy.sleep(0.5)
            result = "False"
            return result
        else:
            print("Yes we can")
            tts.say("ドアが空きました")
            rospy.sleep(0.5)
            result = "True"
            return result
