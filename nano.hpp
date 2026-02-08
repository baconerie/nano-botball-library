/*
Nano is botball library. extends libkipr, providing wrapper functions and  extra functionality
thread safe

you gotta be familiar with object oriented programming to an extent, and basic
programming to understand this 

docs will help you the rest of the way
*/

#pragma once

#include <kipr/libkipr.h>
#include <queue>


namespace Nano {
    void start_nano();

    void wait_for_milliseconds(int milliseconds);
    
    enum WOMBAT_AXIS {
        X_AXIS = 0,
        Y_AXIS = 1,
        Z_AXIS = 2,
    };

    class BaseRobot {
        public:
            BaseRobot();

            void wait_for_light(int light_sensor_pin);

            // Motors
            int get_motor_position_counter(int motor);
            void clear_motor_position_counter(int motor);
            void set_motor_power(int motor, int percent);
            void move_at_velocity(int motor, int velocity);
            void move_to_position(int motor, int speed, int goal_ticks);
            void move_relative_position(int motor, int speed, int delta_ticks);
            bool is_motor_done(int motor);
            void wait_until_motor_done(int motor); // NOTE: do not directly wrap for this one; main thread CANNOT BLOCK!
            void freeze(int motor);
            void freeze_all_motors();

            // Servos. Get servo position is not added because it is unnecessary
            // and may cause confusion.
            bool is_servo_enabled(int port);
            void set_servo_enabled(int port, bool enabled);
            void set_all_servos_enabled(bool enabled);
            void set_servo_position(int port, int position);

            // Analog and digital
            int get_analog(int port);
            int get_digital(int port);
            void set_digital(int port, int value);
            void change_digital_port_mode(int port, bool is_input);

            // IMU
            void calibrate_gyroscope(
                int x_axis_offset_degrees, 
                int y_axis_offset_degrees,
                int z_axis_offset_degrees,
                int num_samples=500
            );
            void calibrate_accelerometer(
                int x_axis_offset_degrees,
                int y_axis_offset_degrees,
                int z_axis_offset_degrees,
                int num_samples=500
            );

            void set_gyroscope_axes(WOMBAT_AXIS x_axis, WOMBAT_AXIS y_axis, WOMBAT_AXIS z_axis);
            void set_accelerometer_axes(WOMBAT_AXIS x_axis, WOMBAT_AXIS y_axis, WOMBAT_AXIS z_axis);
            void set_gyroscope_conversion(double gyro_x_conversion, double gyro_y_conversion, double gyro_z_conversion);
            void set_accelerometer_conversion(double accel_x_conversion, double accel_y_conversion, double accel_z_conversion);

            double get_angular_velocity_x();
            double get_angular_velocity_y();
            double get_angular_velocity_z();

            double get_rotation_x();
            double get_rotation_y();
            double get_rotation_z();

            double get_acceleration_x();
            double get_acceleration_y();
            double get_acceleration_z();

            double get_velocity_x();
            double get_velocity_y();
            double get_velocity_z();

            double get_position_x();
            double get_position_y();
            double get_position_z();

            void TEST_PROGRAM_get_gyroscope_conversion();

            // Advanced functions that most teams will likely not use.
            void get_pid_gains(int motor, short& p, short& i, short& d, short& pd, short& id, short& dd);
            void set_pid_gains(int motor, short p, short i, short d, short pd, short id, short dd);
            int getpwm(int motor); 
            void setpwm(int motor, int pwm);

        private:
            int _imu_last_update_time;

            int _smoothing_buffer_size;
            
            std::queue<int> _gyro_x_smoothing_buffer;
            float _gyro_x_smoothing_buffer_sum;
            std::queue<int> _gyro_y_smoothing_buffer;
            float _gyro_y_smoothing_buffer_sum;
            std::queue<int> _gyro_z_smoothing_buffer;
            float _gyro_z_smoothing_buffer_sum;

            double _gyro_x_conversion;
            double _gyro_y_conversion;
            double _gyro_z_conversion;

            int _gyro_x_bias;
            int _gyro_y_bias;
            int _gyro_z_bias;

            int _gyro_x_threshold;
            int _gyro_y_threshold;
            int _gyro_z_threshold;

            double _rotation_x;
            double _rotation_y;
            double _rotation_z;

            std::queue<int> _accel_x_smoothing_buffer;
            float _accel_x_smoothing_buffer_sum;
            std::queue<int> _accel_y_smoothing_buffer;
            float _accel_y_smoothing_buffer_sum;
            std::queue<int> _accel_z_smoothing_buffer;
            float _accel_z_smoothing_buffer_sum;

            double _accel_x_conversion;
            double _accel_y_conversion;
            double _accel_z_conversion;

            double _accel_x_bias;
            double _accel_y_bias;
            double _accel_z_bias;

            double _accel_x_threshold;
            double _accel_y_threshold;
            double _accel_z_threshold;

            double _velocity_x;
            double _velocity_y;
            double _velocity_z;

            double _position_x;
            double _position_y;
            double _position_z;

    };
    
    /// Wraps an object, allowing thread safe read and write.
    class Mutex {
        public:
            Mutex();
        private:
            mutex _m;
    };

    class MutexLock {
        public:
            MutexLock(Mutex& mutex);
            ~MutexLock();
            void unlock();
        private:
            bool _has_already_unlocked;
            Mutex& _m;
    };

    template<typename Func>
    class Thread {
        public:
            Thread(Func function, bool start_automatically=true);
            void start();
            void wait_for_thread();
            void stop();
        private:
            thread _t;
    };

    namespace Testing {
        // single funnctions which have an entire tesing program built in them.
    };
}
