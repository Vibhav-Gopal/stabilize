#include <iostream>
#include <vector>
#include "tinyxml2.h"
#include "thruster_variables.h"
#include "ros/package.h"

int number_of_thrusters, max_thrust, min_thrust, full_thrust, zero_thrust_pwm, min_pwm, max_pwm;
std::vector<float> surge_vector_init, sway_vector_init, heave_vector_init, yaw_vector_init, pitch_vector_init, roll_vector_init;

void loadThrusterValues() {
    //Opening XML file to parse
    tinyxml2::XMLDocument doc;
    std::string path = ros::package::getPath("testcode") + "/src/thrustered_vehicle_motioncontroller-main/vehicle_config.xml";
    doc.LoadFile(path.c_str());
    //Specifying Root Elements
    auto ThrusterSpecifications = doc.FirstChildElement("vehicle")->FirstChildElement("Thruster_Specifications");
    auto ThrusterConfiguration = doc.FirstChildElement("vehicle")->FirstChildElement("Thruster_Configuration");
    //Assigning Thruster Specification Values
    number_of_thrusters = std::stoi(ThrusterSpecifications->FirstChildElement("No_Of_Thrusters")->GetText());
    min_pwm = std::stoi(ThrusterSpecifications->FirstChildElement("Min_PWM")->GetText());
    max_pwm = std::stoi(ThrusterSpecifications->FirstChildElement("Max_PWM")->GetText());
    zero_thrust_pwm = std::stoi(ThrusterSpecifications->FirstChildElement("Zero_Thrust_PWM")->GetText());
    min_thrust = std::stoi(ThrusterSpecifications->FirstChildElement("Min_Thrust")->GetText());
    max_thrust = std::stoi(ThrusterSpecifications->FirstChildElement("Max_Thrust")->GetText());
    full_thrust = std::stoi(ThrusterSpecifications->FirstChildElement("Full_Thrust")->GetText());
    //Assigning Thruster Configuration Values
    //For Surge Vector
    std::string vector = ThrusterConfiguration->FirstChildElement("Surge_Vector")->GetText();
    std::string value = "";
    int i=1;
    while(vector[i]!='}'){
        if(vector[i]!=','){
            value+=vector[i];
        }
        else{
            surge_vector_init.push_back(std::stof(value));
            value = "";
        }
        i++;
    }
    surge_vector_init.push_back(std::stof(value));
    //For Sway Vector
    vector = ThrusterConfiguration->FirstChildElement("Sway_Vector")->GetText();
    value = "";
    i=1;
    while(vector[i]!='}'){
        if(vector[i]!=','){
            value+=vector[i];
        }
        else{
            sway_vector_init.push_back(std::stof(value));
            value = "";
        }
        i++;
    }
    sway_vector_init.push_back(std::stof(value));
    //For Heave Vector
    vector = ThrusterConfiguration->FirstChildElement("Heave_Vector")->GetText();
    value = "";
    i=1;
    while(vector[i]!='}'){
        if(vector[i]!=','){
            value+=vector[i];
        }
        else{
            heave_vector_init.push_back(std::stof(value));
            value = "";
        }
        i++;
    }
    heave_vector_init.push_back(std::stof(value));
    //For Yaw Vector
    vector = ThrusterConfiguration->FirstChildElement("Yaw_Vector")->GetText();
    value = "";
    i=1;
    while(vector[i]!='}'){
        if(vector[i]!=','){
            value+=vector[i];
        }
        else{
            yaw_vector_init.push_back(std::stof(value));
            value = "";
        }
        i++;
    }
    yaw_vector_init.push_back(std::stof(value));
    //For Pitch Vector
    vector = ThrusterConfiguration->FirstChildElement("Pitch_Vector")->GetText();
    value = "";
    i=1;
    while(vector[i]!='}'){
        if(vector[i]!=','){
            value+=vector[i];
        }
        else{
            pitch_vector_init.push_back(std::stof(value));
            value = "";
        }
        i++;
    }
    pitch_vector_init.push_back(std::stof(value));
    //For Roll Vector
    vector = ThrusterConfiguration->FirstChildElement("Roll_Vector")->GetText();
    value = "";
    i=1;
    while(vector[i]!='}'){
        if(vector[i]!=','){
            value+=vector[i];
        }
        else{
            roll_vector_init.push_back(std::stof(value));
            value = "";
        }
        i++;
    }
    roll_vector_init.push_back(std::stof(value));
}

void printThrusterValues(){
    int i;
    std::cout<<number_of_thrusters<<' '<<min_pwm<<' '<<max_pwm<<' '<<zero_thrust_pwm<<' '<<min_thrust<<' '<<max_thrust<<' '<<full_thrust<<'\n';
    std::cout<<'{';
    for(i=0;i<surge_vector_init.size()-1;i++){
        std::cout<<surge_vector_init[i]<<',';
    }
    std::cout<<surge_vector_init[surge_vector_init.size()-1]<<"}  ";
    std::cout<<'{';
    for(i=0;i<sway_vector_init.size()-1;i++){
        std::cout<<sway_vector_init[i]<<',';
    }
    std::cout<<sway_vector_init[sway_vector_init.size()-1]<<"}  ";
    std::cout<<'{';
    for(i=0;i<heave_vector_init.size()-1;i++){
        std::cout<<heave_vector_init[i]<<',';
    }
    std::cout<<heave_vector_init[heave_vector_init.size()-1]<<"}  ";
    std::cout<<'{';
    for(i=0;i<yaw_vector_init.size()-1;i++){
        std::cout<<yaw_vector_init[i]<<',';
    }
    std::cout<<yaw_vector_init[yaw_vector_init.size()-1]<<"}  ";
    std::cout<<'{';
    for(i=0;i<pitch_vector_init.size()-1;i++){
        std::cout<<pitch_vector_init[i]<<',';
    }
    std::cout<<pitch_vector_init[pitch_vector_init.size()-1]<<"}  ";
    std::cout<<'{';
    for(i=0;i<roll_vector_init.size()-1;i++){
        std::cout<<roll_vector_init[i]<<',';
    }
    std::cout<<roll_vector_init[roll_vector_init.size()-1]<<"}\n";
}