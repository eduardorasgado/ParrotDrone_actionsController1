import rospy
from geometry_msgs.msg import Twist

class DroneFlying:
    def __init__(self, topic="/cmd_vel"):
        self.publishertoFly = rospy.Publisher(topic, Twist, queue_size=0)
    
    def go(self):
        #PUBLISHER TO START THE DRONE ENGINE!
        fly = Twist()
        fly.linear.x = 1
        fly.angular.y = 0.2

        self.publishertoFly.publish(fly)
        
    def stop(self):
        fly = Twist()
        fly.linear.x = 0
        fly.angular.y = 0
        
        self.publishertoFly.publish(fly)