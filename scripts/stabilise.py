#!/usr/bin/env python3
import sys
sys.path.append('/home/vibhav/localization_ws/devel/lib/')
import motion_controller as m
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation


bot = m.ThrusteredVehicleMotionController()
bot.setHeaveControlMode(0) # 0 means closed loop mode
bot.setSurgeControlMode(0) # 0 means closed loop mode
bot.setSwayControlMode(0) # 0 means closed loop mode

bot.setYawControlMode(0) # 0 means closed loop mode
bot.setPitchControlMode(0) # 0 means closed loop mode
bot.setRollControlMode(0) # 0 means closed loop mode

bot.resetAllThrusters()

bot.setTargetRollAngle(0)
bot.setTargetPitchAngle(0)
bot.setTargetYawAngle(0)

bot.setTargetHeavePoint(0)
bot.setTargetSwayPoint(0)
bot.setTargetSurgePoint(0)

#TODO SET PID CONSTANTS

def updateStateCb(data):
    quarternion = [data.pose.pose.orientation.x,
                   data.pose.pose.orientation.y,
                   data.pose.pose.orientation.z,
                   data.pose.pose.orientation.w]
    rot = Rotation.from_quat(quarternion)
    euler = rot.as_euler("xyz",degrees=True)
    bot.updateCurrentPitchAngle(euler[1])
    bot.updateCurrentRollAngle(euler[0])
    bot.updateCurrentYawAngle(euler[2])
    bot.updateCurrentHeavePoint(data.pose.pose.position.z)
    bot.updateCurrentSurgePoint(data.pose.pose.position.x)
    bot.updateCurrentSwayPoint(data.pose.pose.position.y)
    bot.updateThrustValues()
    bot.refresh()
def setTargetCb(data):
    ...

rospy.init_node("Bot")
rospy.Subscriber("/odometry/filtered",Odometry,updateStateCb) #Current state updater
#TODO rospy.Subscriber("/targetPose",Pose,setTargetCb)
rospy.spin()