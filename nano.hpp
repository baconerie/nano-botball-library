/*
Nano is botball library. extends libkipr, providing wrapper functions and  extra functionality
thread safe

you gotta be familiar with object oriented programming to an extent, and basic
programming to understand this 

docs will help you the rest of the way
*/


namespace Nano {
    void start_nano();

    void wait_for_milliseconds(int milliseconds);
    
    class BaseRobot {
        public:
            BaseRobot();

            void wait_for_light(int light_sensor_pin);

            // Motors
            int get_motor_position_counter(int motor);
            void clear_motor_position_counter(int motor);
            int set_motor_power(int motor, int percent);
            int move_at_velocity(int motor, int velocity);
            void move_to_position(int motor, int speed, int goal_ticks);
            void move_relative_position(int motor, int speed, int delta_ticks);
            bool is_motor_done(int motor);
            void wait_until_motor_done(int motor); // NOTE: do not directly wrap for this one; main thread CANNOT BLOCK!
            void freeze(int motor);

            // Servos. Get servo position is not added because it is unnecessary
            // and may cause confusion.
            bool is_servo_enabled(int port);
            void set_servo_enabled(int port, bool enabled);
            void set_all_servos_enabled(bool enabled);
            void set_servo_position(int port, int position);

            // Advanced functions that most teams will likely not use.
            int set_pid_gains(int motor, short p, short i, short d, short pd, short id, short dd);
            int get_pid_gains(int motor, short& p, short& i, short& d, short& pd, short& id, short& dd);
            int setpwm(int motor, int pwm);
            int getpwm(int motor);

            // Analog and digital
            int get_analog(int port);
            int get_digital(int port);
            int set_digital(int port, int value);
            int change_digital_port_mode(int port, bool is_input);

            // Gyroscope
            int get_gyro_x();
            int get_gyro_y();
            int get_gyro_z();   
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
            start();
            wait_for_thread();
            stop();
        private:
            thread _t;
    };

    namespace Testing {
        // single funnctions which have an entire tesing program built in them.
    }
}
