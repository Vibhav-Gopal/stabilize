#include "thrustered_vehicle_motion_controller.h"
#include  "PID-Controller/PID_controller.h"
#include  "thrusters_controller/thruster_variables.h"
#include  "thrusters_controller/thrusters_controller.h"
#include  <iostream>

ThrusteredVehicleMotionController::ThrusteredVehicleMotionController(/* args */){

    ThrustersController::init();

    surge_control_mode = OPEN_LOOP_MODE;
    surge_controller.setConstants(1, 1, 1, 0.001);
    surge_controller.setMinMaxLimits(-100, 100, -50, 50);
    
    sway_control_mode = OPEN_LOOP_MODE;
    sway_controller.setConstants(1, 1, 1, 0.001);
    sway_controller.setMinMaxLimits(-100, 100, -50, 50);
    
    heave_control_mode = CLOSED_LOOP_MODE;
    heave_controller.setConstants(1, 1, 1, 0.001);
    heave_controller.setMinMaxLimits(-100, 100, -50, 50);

    yaw_control_mode = CLOSED_LOOP_MODE;
    yaw_controller.setConstants(1, 1, 1, 0.001);
    yaw_controller.setMinMaxLimits(-100, 100, -50, 50);

    pitch_control_mode = CLOSED_LOOP_MODE;
    pitch_controller.setConstants(1, 1, 1, 0.001);
    pitch_controller.setMinMaxLimits(-100, 100, -50, 50);
    
    roll_control_mode = CLOSED_LOOP_MODE;
    roll_controller.setConstants(1, 1, 1, 0.001);
    roll_controller.setMinMaxLimits(-100, 100, -50, 50);

    surge_thrust = sway_thrust = heave_thrust = yaw_thrust = pitch_thrust = roll_thrust = 0;
    surge_vector = surge_vector_init; sway_vector = sway_vector_init; heave_vector = heave_vector_init;
    yaw_vector = yaw_vector_init; pitch_vector = pitch_vector_init; roll_vector = roll_vector_init;
    
    for (int i = 0; i < number_of_thrusters; i++)
    {
        thrust_vector.push_back(0);
    }
    
}
ThrusteredVehicleMotionController::~ThrusteredVehicleMotionController(/* args */){
    ThrustersController::shutdown();
}

void ThrusteredVehicleMotionController::setSurgeControlMode(bool mode){
    surge_control_mode = mode;

    if (mode == CLOSED_LOOP_MODE)
    {
        surge_controller.reset();
    }
    
}

void ThrusteredVehicleMotionController::setSwayControlMode(bool mode){
    sway_control_mode = mode;

     if (mode == CLOSED_LOOP_MODE)
    {
        sway_controller.reset();
    }
}

void ThrusteredVehicleMotionController::setHeaveControlMode(bool mode){
    heave_control_mode = mode;

     if (mode == CLOSED_LOOP_MODE)
    {
        heave_controller.reset();
    }
}

void ThrusteredVehicleMotionController::setYawControlMode(bool mode){
    yaw_control_mode = mode;

     if (mode == CLOSED_LOOP_MODE)
    {
        yaw_controller.reset();
    }
}

void ThrusteredVehicleMotionController::setPitchControlMode(bool mode){
    pitch_control_mode = mode;

     if (mode == CLOSED_LOOP_MODE)
    {
        pitch_controller.reset();
    }
}

void ThrusteredVehicleMotionController::setRollControlMode(bool mode){
    roll_control_mode = mode;

     if (mode == CLOSED_LOOP_MODE)
    {
        roll_controller.reset();
    }
}

void ThrusteredVehicleMotionController::setSurgePIDConstants(float kp,float ki, float kd, float acceptable_error){
    
    surge_controller.setConstants(kp, ki, kd, acceptable_error);

}

void ThrusteredVehicleMotionController::setSwayPIDConstants(float kp,float ki, float kd, float acceptable_error){
    
    sway_controller.setConstants(kp, ki, kd, acceptable_error);

}

void ThrusteredVehicleMotionController::setHeavePIDConstants(float kp,float ki, float kd, float acceptable_error){
    
    heave_controller.setConstants(kp, ki, kd, acceptable_error);

}

void ThrusteredVehicleMotionController::setYawPIDConstants(float kp,float ki, float kd, float acceptable_error){
    
    yaw_controller.setConstants(kp, ki, kd, acceptable_error);

}

void ThrusteredVehicleMotionController::setPitchPIDConstants(float kp,float ki, float kd, float acceptable_error){
    
    pitch_controller.setConstants(kp, ki, kd, acceptable_error);

}

void ThrusteredVehicleMotionController::setRollPIDConstants(float kp,float ki, float kd, float acceptable_error){
    
    roll_controller.setConstants(kp, ki, kd, acceptable_error);

}


void ThrusteredVehicleMotionController::setSurgePIDLimits(float output_min,float output_max, float integral_min, float integral_max){
    surge_controller.setMinMaxLimits(output_min,output_max,integral_min,integral_max);
}


void ThrusteredVehicleMotionController::setSwayPIDLimits(float output_min,float output_max, float integral_min, float integral_max){
    sway_controller.setMinMaxLimits(output_min,output_max,integral_min,integral_max);
}

void ThrusteredVehicleMotionController::setHeavePIDLimits(float output_min,float output_max, float integral_min, float integral_max){
    heave_controller.setMinMaxLimits(output_min,output_max,integral_min,integral_max);
}

void ThrusteredVehicleMotionController::setYawPIDLimits(float output_min,float output_max, float integral_min, float integral_max){
    yaw_controller.setMinMaxLimits(output_min,output_max,integral_min,integral_max);
}

void ThrusteredVehicleMotionController::setPitchPIDLimits(float output_min,float output_max, float integral_min, float integral_max){
    pitch_controller.setMinMaxLimits(output_min,output_max,integral_min,integral_max);
}

void ThrusteredVehicleMotionController::setRollPIDLimits(float output_min,float output_max, float integral_min, float integral_max){
    roll_controller.setMinMaxLimits(output_min,output_max,integral_min,integral_max);
}


void ThrusteredVehicleMotionController::setTargetSurgePoint(float point){
    if (surge_control_mode == CLOSED_LOOP_MODE)
    {
        surge_controller.setTargetValue(point);
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }
    

}


void ThrusteredVehicleMotionController::setTargetSwayPoint(float point){
    if (sway_control_mode == CLOSED_LOOP_MODE)
    {
        sway_controller.setTargetValue(point);
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }
    

}

void ThrusteredVehicleMotionController::setTargetHeavePoint(float point){
    if (heave_control_mode == CLOSED_LOOP_MODE)
    {
        heave_controller.setTargetValue(point);
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }
    

}

void ThrusteredVehicleMotionController::setTargetYawAngle(float angle){
    if (yaw_control_mode == CLOSED_LOOP_MODE)
    {
        yaw_controller.setTargetValue(angle);
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }
    

}

void ThrusteredVehicleMotionController::setTargetPitchAngle(float angle){
    if (pitch_control_mode == CLOSED_LOOP_MODE)
    {
        pitch_controller.setTargetValue(angle);
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }
    

}

void ThrusteredVehicleMotionController::setTargetRollAngle(float angle){
    if (roll_control_mode == CLOSED_LOOP_MODE)
    {
        roll_controller.setTargetValue(angle);
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }
    

}

void ThrusteredVehicleMotionController::updateCurrentSurgePoint(float point){

    if (surge_control_mode == CLOSED_LOOP_MODE)
    {
         surge_controller.setCurrentValue(point);
         surge_thrust =  surge_controller.updateOutput();
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }

}


void ThrusteredVehicleMotionController::updateCurrentSwayPoint(float point){

    if (sway_control_mode == CLOSED_LOOP_MODE)
    {
         sway_controller.setCurrentValue(point);
         sway_thrust =  sway_controller.updateOutput();
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }

}

void ThrusteredVehicleMotionController::updateCurrentHeavePoint(float point){

    if (heave_control_mode == CLOSED_LOOP_MODE)
    {
         heave_controller.setCurrentValue(point);
         heave_thrust =  heave_controller.updateOutput();
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }

}

void ThrusteredVehicleMotionController::updateCurrentYawAngle(float angle){

    if (yaw_control_mode == CLOSED_LOOP_MODE)
    {
         yaw_controller.setCurrentValue(angle);
         yaw_thrust =  yaw_controller.updateOutput();
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }

}

void ThrusteredVehicleMotionController::updateCurrentPitchAngle(float angle){

    if (pitch_control_mode == CLOSED_LOOP_MODE)
    {
         pitch_controller.setCurrentValue(angle);
         pitch_thrust =  pitch_controller.updateOutput();
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }

}

void ThrusteredVehicleMotionController::updateCurrentRollAngle(float angle){

    if (roll_control_mode == CLOSED_LOOP_MODE)
    {
         roll_controller.setCurrentValue(angle);
         roll_thrust =  roll_controller.updateOutput();
    }
    else
    {
        std::cout<<"Error, closed loop control not enabled"<<std::endl;
    }

}


void ThrusteredVehicleMotionController::setSurgeThrust(float thrust){
     if (surge_control_mode != CLOSED_LOOP_MODE)
    {
         
         surge_thrust =  thrust;
    }
    else
    {
        std::cout<<"Error, closed loop control enabled, cannot  set thrust manually."<<std::endl;
    }

}

void ThrusteredVehicleMotionController::setSwayThrust(float thrust){
     if (sway_control_mode != CLOSED_LOOP_MODE)
    {
         
         sway_thrust =  thrust;
    }
    else
    {
        std::cout<<"Error, closed loop control enabled, cannot  set thrust manually."<<std::endl;
    }

}

void ThrusteredVehicleMotionController::setHeaveThrust(float thrust){
     if (heave_control_mode != CLOSED_LOOP_MODE)
    {
         
         heave_thrust =  thrust;
    }
    else
    {
        std::cout<<"Error, closed loop control enabled, cannot  set thrust manually."<<std::endl;
    }

}


void ThrusteredVehicleMotionController::setYawThrust(float thrust){
     if (yaw_control_mode != CLOSED_LOOP_MODE)
    {
         
         yaw_thrust =  thrust;
    }
    else
    {
        std::cout<<"Error, closed loop control enabled, cannot  set thrust manually."<<std::endl;
    }

}

void ThrusteredVehicleMotionController::setPitchThrust(float thrust){
     if (pitch_control_mode != CLOSED_LOOP_MODE)
    {
         
         pitch_thrust =  thrust;
    }
    else
    {
        std::cout<<"Error, closed loop control enabled, cannot  set thrust manually."<<std::endl;
    }

}


void ThrusteredVehicleMotionController::setRollThrust(float thrust){
     if (roll_control_mode != CLOSED_LOOP_MODE)
    {
         
         roll_thrust =  thrust;
    }
    else
    {
        std::cout<<"Error, closed loop control enabled, cannot  set thrust manually."<<std::endl;
    }

}

void ThrusteredVehicleMotionController::resetAllThrusters(){
    surge_thrust = sway_thrust = heave_thrust = yaw_thrust = pitch_thrust = roll_thrust = 0;
}

void ThrusteredVehicleMotionController::refresh(){
    ThrustersController::refresh();
}

 void ThrusteredVehicleMotionController::updateThrustValues(){

    for (int i = 0; i < number_of_thrusters; i++)
    {
        thrust_vector[i] = (surge_thrust*surge_vector[i]) + (sway_thrust*sway_vector[i]) + (heave_thrust*heave_vector[i]) + (yaw_thrust*yaw_vector[i]) +  (pitch_thrust*pitch_vector[i]) + (roll_thrust*roll_vector[i]);
        thrust_vector[i] = limitToRange(thrust_vector[i],min_thrust,max_thrust);
    }
    float* thrust_array = &thrust_vector[0];
    ThrustersController::writeThrusterValues(thrust_array);
 }


float ThrusteredVehicleMotionController::limitToRange(float value, float minimum, float maximum){
    if (value > maximum)
    {
        return maximum;
    }
    else if (value < minimum)
    {
        return minimum;
    }
    else
    {
        return value;
    }
    
    
    
}
