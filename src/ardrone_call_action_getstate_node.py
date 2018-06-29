#! /usr/bin/env python
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback
from std_msgs.msg import Empty

#the fly module to publish to /cmd_vel
from flyingDrone_publisher import DroneFlying
"""
For calling the drone to fly quiet:
rostopic pub /drone/takeoff std_msgs/Empty "{}"

For calling the drone to lay down the ground:
rostopic pub /drone/land std_msgs/Empty "{}"
"""

PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

class upanddown:
    def __init__(self, takeoff = None, land = None):
        self.takeoff = takeoff
        self.land = land
        self.takeoff_msg = Empty() #Create the message to takeoff the drone
        self.land_msg = Empty() #Create the message to land the drone
        
    def up(self):
        self.takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1) #Create a Publisher to takeoff the drone
        
        
    def down(self):
        self.land = rospy.Publisher('/drone/land', Empty, queue_size=1) #Create a Publisher to land the drone
        
        
    def publish(self,choice):
        if choice == 1:
            self.takeoff.publish(self.takeoff_msg)
        if choice == 0:
            self.land.publish(self.land_msg)

class Fly:
    
    def __init__(self, secondstofly=10, action_server_name = '/ardrone_action_server', action=ArdroneAction, nImage = 1):
        self.secondstofly = secondstofly
        self.action_server_name = action_server_name
        self.action = action
        self.nImage = nImage
        self.nodo = rospy.init_node('example_no_waitforresult_action_client_node')
        self.client = actionlib.SimpleActionClient(action_server_name, self.action)
        
        self.setup = upanddown()
        self.setup.up()
        self.setup.down()
        
        self.waitingforServer(self.client)
        self.goalHandling(self.client, self.secondstofly)
        
        #state_result will give the final state
        #Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
        self.state_result = self.client.get_state()
        
        self.rate = rospy.Rate(1)
        self.publishertoFly = None
        
        #create the object to handle the drone going forward and stoping
        self.mydroneFly = DroneFlying()
        
        self.taking_off_land_the_drone(self.setup, 1)
        
        self.state_result, self.mydroneFly = self.theflyAndPhoto(self.state_result, self.mydroneFly, self.rate, self.client)
        
        #stop the drone flying foward and landing the drone
        self.mydroneFly.stop()
        self.taking_off_land_the_drone(self.setup, 0)
        
        self.loginfo("[Result] State: ",str(self.state_result))
        
        if self.state_result == ERROR:
            self.logerror("Something went wrong in the server")
        if self.state_result == WARN:
            self.logwarn("There is a warning in the server")
        
    def taking_off_land_the_drone(self, setup, up_or_down):
        #We takeoff the drone during the first 3 seconds
        i=0
        while not i == 3:
            #taking off the drone
            setup.publish(up_or_down)
            rospy.loginfo('Taking off...')
            time.sleep(1)
            i += 1
        
    def waitingforServer(self, client):
        self.loginfo("Waiting for action Server", self.action_server_name)
        client.wait_for_server()
        self.loginfo("Action Server found:", self.action_server_name)
        
    def goalHandling(self, client,secondstofly):
        goal = ArdroneGoal()
        goal.nseconds = secondstofly
        client.send_goal(goal, feedback_cb=self.feedback_callback)
        
    def theflyAndPhoto(self,state_result, drone, rate, client):
        while state_result < DONE:
            drone.go()
            rate.sleep()
            state_result = client.get_state()
            self.loginfo("state_result: ", str(state_result))
        return [state_result, drone]
        
    def feedback_callback(self, feedback):
        print('[Feedback] image n.%d received'%self.nImage)
        self.nImage += 1
    
    def logerror(self,message):
        rospy.logerr(message)
    
    def logwarn(self, message):
        rospy.logwarn(message)
    
    def loginfo(self, message1, message2):
        rospy.loginfo(message1+message2)
        
        
if __name__ == "__main__":
    
    photoFly = Fly(secondstofly=20)