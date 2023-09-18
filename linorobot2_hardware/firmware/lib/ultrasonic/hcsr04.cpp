#ifndef _HCSR04_
#define _HCSR04_

#include <Arduino.h>
#include <sensor_msgs/msg/range.h>

class HCSR04
{
    private:
        sensor_msgs__msg__Range range_msg_;
        const float g_to_accel_ = 9.81;
        geometry_msgs__msg__Vector3 accel_;

    public:
        HCSR04()
        {
            range_msg_.header.frame_id = micro_ros_string_utilities_set(range_msg_.header.frame_id, "sonic1_link");
        }
        geometry_msgs__msg__Vector3 readAccelerometer() = 0;
        bool startSensor() = 0;

        bool init()
        {
            bool sensor_ok = startSensor();
            // if(sensor_ok)
            //     calibrateGyro();

            return sensor_ok;
        }

        sensor_msgs__msg__Imu getData()
        {
            digitalWrite(trigPin, LOW);
            delayMicroseconds(5);
            digitalWrite(trigPin, HIGH);
            delayMicroseconds(10);
            digitalWrite(trigPin, LOW);
           
            pinMode(echoPin, INPUT);
            duration = pulseIn(echoPin, HIGH);
            cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
      //
      //
            imu_msg_.angular_velocity = readGyroscope();
            imu_msg_.angular_velocity.x -= gyro_cal_.x;
            imu_msg_.angular_velocity.y -= gyro_cal_.y;
            imu_msg_.angular_velocity.z -= gyro_cal_.z;

            if(imu_msg_.angular_velocity.z > -0.01 && imu_msg_.angular_velocity.z < 0.01 )
                imu_msg_.angular_velocity.z = 0;

            imu_msg_.angular_velocity_covariance[0] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[4] = gyro_cov_;
            imu_msg_.angular_velocity_covariance[8] = gyro_cov_;

            imu_msg_.linear_acceleration = readAccelerometer();
            imu_msg_.linear_acceleration_covariance[0] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[4] = accel_cov_;
            imu_msg_.linear_acceleration_covariance[8] = accel_cov_;


            return range_msg_;
            // return imu_msg_;
        }

        bool startSensor()
        {
            int trigPin = 22;    // Trigger
            int echoPin = 23;    // Echo
            long duration, cm, inches;
            pinMode(trigPin, OUTPUT);
            pinMode(echoPin, INPUT);

            // Wire.begin();
            // bool ret;
            // accelerometer_.initialize();
            // ret = accelerometer_.testConnection();
            // if(!ret)
            //     return false;

            return true;
        }
};

#endif
