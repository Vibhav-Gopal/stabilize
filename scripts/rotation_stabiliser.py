#!/usr/bin/env python3
# TODO
# refractor code to have yaw roll pitch updating in same method
# Add code to set PID constants
# Tune PID constants
# Line 108
# Pitch

# dpad up - pitch hold
# dpad left - pitch reset
# dpad right - yaw hold

import sys
from time import sleep
sys.path.append('/home/vibhav/localization_ws/devel/lib/')
import motion_controller as m
import rospy
from std_msgs.msg import String,Float32
from materov22_pioneer.msg import controller
bot = m.ThrusteredVehicleMotionController()
reset=0
bot.setHeaveControlMode(1)
bot.setSurgeControlMode(1)
bot.setSwayControlMode(1)

bot.setYawControlMode(1)
bot.setRollControlMode(1)
bot.setPitchControlMode(1)

bot.resetAllThrusters()

# bot.setTargetYawAngle(0)

# bot.setTargetPitchAngle(0)


angle_sensitivity_yaw = 30

angle_sensitivity_pitch = 29

angle_sens_roll = 27

yaw_joystick:int
roll_joystick:int
pitch_joystick:int

roll_joystick=0
#DONT INCREASE BEYONG 32
multiplier_heave = 32
multiplier_sway = 31
multiplier_surge = 31


buffer_multiplier = 0.09
current_angle_yaw = current_angle_roll = current_angle_pitch = 0 # values to rotate the bot to, input from joystick

hold_pitch=False #This variable is active low, when active, it holds the pitch angle, it is active by default
hold_yaw = False
tmp=0
tmp_yaw=0

def limitVals(val):
    return max(min(0.2,val),-0.2)
#help(m)
def mul(x):
    return 100*x
def call(data):
    global hold_pitch, current_angle_yaw, current_angle_pitch,tmp,tmp_yaw,hold_yaw
    # print('hi')
    
    # lis = data.data.split('_')
    # lis = list(map(float,lis))
    # lis = list(map(mul,lis))
    # lis[5] = lis[5]+100
    # lis[5]  = lis[5]/2
    # lis[5] = round(lis[5])
    # lis[5] = -1*lis[5]
    
    # lis[4] = lis[4]+100
    # lis[4]  = lis[4]/2
    # lis[4] = round(lis[4])
    
    # lis[1] = -1*lis[1]
    
    # thrust = lis[4]+lis[5]
    # surge = lis[1]
    # yaw = lis[2]
    
    # reset = lis[10]

## Left joy front - forward
## Left joy left - translate left
## Right joy front - pitch down
## Right joy left - yaw left

    heave_up = data.right_trigger #right trigger moves bot upwards towards surface
    heave_down = data.left_trigger
    heave = heave_up - heave_down
    
    sway = data.vals.ax0 

    surge = data.vals.ax1
    surge*=-1 #Surge joy is reversed
    

    yaw_joystick = data.vals.ax2

    pitch_joystick = data.vals.ax3

    roll_joystick_left = data.left_bumper
    roll_joystick_right = data.right_bumper
    
    roll_joystick = roll_joystick_left - roll_joystick_right

    if(data.pitch_hold==1):
        if(tmp==0):
            hold_pitch=~hold_pitch
        tmp = data.pitch_hold
    else:
        tmp=data.pitch_hold

    if ~hold_pitch : print("Holding pitch")
    else: print("Pitch released")

    if(data.yaw_hold==1):
        if(tmp_yaw==0):
            hold_yaw=~hold_yaw
        tmp_yaw = data.yaw_hold
    else:
        tmp_yaw=data.yaw_hold

    if hold_yaw : print("Holding yaw")
    else: print("Yaw released")




    if(reset ==1):
        bot.resetAllThrusters()

    # heave=limitVals(heave)
    # sway=limitVals(sway)
    # surge=limitVals(surge)

    heave*=multiplier_heave
    sway*=multiplier_sway
    surge*=multiplier_surge
    yaw = yaw_joystick*angle_sensitivity_yaw
    roll = roll_joystick*angle_sens_roll*-1
    pitch = pitch_joystick*angle_sensitivity_pitch
## Within 7% of the signal, the response is zero, so stick drift can be avoided (not the best way)
    if (heave) >= -(buffer_multiplier*multiplier_heave) and (heave) <=(buffer_multiplier*multiplier_heave):
        heave =0
        
    if (surge) >= -(buffer_multiplier*multiplier_surge) and (surge) <= (buffer_multiplier*multiplier_surge):
        surge =0
        
    if (sway) >= -(buffer_multiplier*multiplier_sway) and (sway) <= (buffer_multiplier*multiplier_sway):
        sway =0

## TODO 
## Change below code to buffer angle values
    if (yaw_joystick) >= -(buffer_multiplier) and (yaw_joystick) <=(buffer_multiplier):
        yaw_joystick =0
        
    if (pitch_joystick) >= -(buffer_multiplier) and (pitch_joystick) <= (buffer_multiplier):
        pitch_joystick =0



    bot.setHeaveThrust(heave)
    bot.setSurgeThrust(surge)
    bot.setSwayThrust(sway)
    bot.setYawThrust(yaw)
    bot.setRollThrust(roll)
    bot.setPitchThrust(pitch)
    ## put buffer on input values

    if ~hold_yaw: current_angle_yaw += yaw_joystick*angle_sensitivity_yaw

    
    if hold_pitch:current_angle_pitch += pitch_joystick*angle_sensitivity_pitch

    if(data.pitch_reset == 1):
        current_angle_pitch=0

    # bot.setTargetYawAngle(current_angle_yaw)

    # bot.setTargetPitchAngle(current_angle_pitch)

    bot.updateThrustValues()
    bot.refresh()
    # print("Heave:",heave,"Surge:",surge,"Sway",sway,"Reset-",reset)
    print("Roll",roll,"Yaw",yaw,"Heave",heave,"Surge",surge)


    
    pass
# def rot_yaw(data):
#     bot.updateCurrentYawAngle(data.data)
#     bot.updateThrustValues()
#     bot.refresh()

# def rot_roll(data):
#     bot.updateCurrentRollAngle(data.data)
#     bot.updateThrustValues()
#     bot.refresh()


# def rot_pitch(data):
#     bot.updateCurrentPitchAngle(data.data)
#     bot.updateThrustValues()
#     bot.refresh()



rospy.init_node("Bot")
rospy.Subscriber('/joydata', controller,call)
# rospy.Subscriber('/yaw',Float32,rot_yaw)
# rospy.Subscriber('/roll',Float32,rot_roll)
# rospy.Subscriber('/pitch',Float32,rot_pitch)
# surge = 0.1
# surge*=multiplier_surge
# bot.setSurgeThrust(surge)
# bot.updateThrustValues()
# bot.refresh()
# sleep(3)
# surge = 0
# surge*=multiplier_surge
# bot.setSurgeThrust(surge)
# bot.updateThrustValues()
# bot.refresh()

rospy.spin()
