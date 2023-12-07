#include "thruster_variables.h"
#include "thrusters_controller.h"
#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include <vector>


ros::NodeHandle* nh;
ros::Publisher* pwm_pub;
std_msgs::Int32MultiArray* pwm_msg;
float limitToRange(float value, float minimum, float maximum);

void ThrustersController::init(){
    int argc = 0;
    ros::init(argc,NULL,"thrusters_controller");
    nh = new ros::NodeHandle;
    pwm_pub = new ros::Publisher;
    pwm_msg = new std_msgs::Int32MultiArray;
    *pwm_pub = nh->advertise<std_msgs::Int32MultiArray>("/pwm_values",50);
    
    loadThrusterValues();
    pwm_msg->data.resize(number_of_thrusters);
}


void ThrustersController::writeThrusterValues(float* thrust_vector){
    for (int i = 0; i < number_of_thrusters; i++)
    {   
         
         pwm_msg->data[i] = zero_thrust_pwm + thrust_vector[i]* (max_pwm - zero_thrust_pwm)/full_thrust;
         pwm_msg->data[i] = limitToRange(pwm_msg->data[i],min_pwm,max_pwm);
    }
    pwm_pub->publish(*pwm_msg);
}


void ThrustersController::refresh(){
    ros::spinOnce();
}


void ThrustersController::shutdown(){
    delete nh;
    delete pwm_pub;
    delete pwm_msg;

}


float limitToRange(float value, float minimum, float maximum){
    if (value > maximum){
        return maximum;
    }
    else if (value < minimum){
        return minimum;
    }
    else{
        return value;
    }
}