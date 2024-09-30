from crewai_messages import (CIAgent, BIAgent)
from geometry_msgs.msg import Twist
from pydantic import BaseModel
import rospy
from rospy import Publisher, Subscriber, Rate
from typing import Optional


class CampusInchargeAgentFollower(BaseModel):
    pub_visitor_vel: Optional[Publisher] = None
    sub_cmd_vel: Optional[Subscriber] = None
    halt_follower: bool = False  

    class Config:
        arbitrary_types_allowed = True

    def __init__(self, **data):
        super().__init__(**data)
        self.sub_cmd_vel = Subscriber('campus_incharge_agent/cmd_vel', Twist, self.callback)
        self.pub_visitor_vel = Publisher('visitor_agent/cmd_vel', Twist, queue_size=10)
        rospy.loginfo("[CampusInchargeAgentFollower] Initialized: Listening to 'campus_incharge_agent/cmd_vel' and publishing to 'visitor_agent/cmd_vel'...")

    def callback(self, message: Twist):
        if not self.halt_follower:  
            rospy.loginfo("[CampusInchargeAgentFollower] Relaying velocity command from 'campus_incharge_agent' to 'visitor_agent'...")
            self.pub_visitor_vel.publish(message)
        
    def run(self):
        rate = Rate(20)  
        while not self.halt_follower:
            rate.sleep()  
        rospy.loginfo("[CampusInchargeAgentFollower] Stopped: Follower thread has been halted.")


class BuildingInchargeAgentFollower(BaseModel):
    pub_visitor_vel: Optional[Publisher] = None
    sub_cmd_vel: Optional[Subscriber] = None
    halt_follower: bool = False

    class Config:
        arbitrary_types_allowed = True

    def __init__(self, **data):
        super().__init__(**data)
        self.sub_cmd_vel = Subscriber('building_incharge_agent/cmd_vel', Twist, self.callback)
        self.pub_visitor_vel = Publisher('visitor_agent/cmd_vel', Twist, queue_size=10)
        rospy.loginfo("[BuildingInchargeAgentFollower] Initialized: Listening to 'building_incharge_agent/cmd_vel' and publishing to 'visitor_agent/cmd_vel'...")

    def callback(self, message: Twist):
        if not self.halt_follower:
            rospy.loginfo("[BuildingInchargeAgentFollower] Relaying velocity command from 'building_incharge_agent' to 'visitor_agent'...")
            self.pub_visitor_vel.publish(message)

    def run(self):
        rate = Rate(20)
        while not self.halt_follower:
            rate.sleep()
        rospy.loginfo("[BuildingInchargeAgentFollower] Stopped: Follower thread has been halted.")