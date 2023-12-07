#ifndef THRUSTERS_CONTROLLER_H
#define THRUSTERS_CONTROLLER_H

namespace ThrustersController{
    void init();
    void writeThrusterValues(float* thrust_vector);
    void refresh();
    void shutdown();
};

#endif