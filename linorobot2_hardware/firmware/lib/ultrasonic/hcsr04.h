#ifndef _HCSR04_H
#define _HCSR04_H

#include <Arduino.h>
#include <sensor_msgs/msg/range.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

class HCSR04
{
    private:
        sensor_msgs__msg__Range range_msg_;
        int trigPin;    // Trigger
        int echoPin;    // Echo
        float ranges[2];
        int duration;
        float distance_m;
    public:
        HCSR04();
        // bool startSensor() = 0;
        bool init();
        sensor_msgs__msg__Range getData();
        bool startSensor();
};

#endif
