#include <nano.hpp>
#include <queue>
#include <tuple>
#include <variant>
#include <iostream>

Nano::Thread worker_thread(worker_thread_function, false);
bool has_already_started_worker_thread = false;


Nano::Mutex message_mutex;
enum MessageType {
    // Motor functions
    get_motor_position_counter,
    clear_motor_position_counter,
    set_motor_power,
    move_at_velocity,
    move_to_position,
    move_relative_position,
    is_motor_done,
    freeze,

    // Servo functions
    is_servo_enabled,
    set_servo_enabled,
    set_all_servos_enabled,
    set_servo_position,

    // Analog and digital functions
    get_analog,
    get_digital,
    set_digital,
    change_digital_port_mode,

    // Gyroscope functions
    get_gyro_x_raw,
    get_gyro_y_raw,
    get_gyro_z_raw,

    // Advanced motor functions
    get_pid_gains,
    set_pid_gains,
    getpwm,
    setpwm,
};

typedef std::variant<
    int,
    std::tuple<int, int>, 
    std::tuple<int, bool>,
    std::tuple<int, int, int>,
    std::tuple<int, short, short, short, short, short, short>,
> MessageContent;
std::queue<std::tuple<unsigned int, MessageType, MessageContent>> request_message_queue;
unsigned int next_message_id = 0;
std::queue<std::tuple<unsigned int, MessageContent>> response_message_queue;

void worker_thread_function() {
    while (true) {
        Nano::wait_for_milliseconds(2);
        Nano::MutexLock lock(message_mutex);

        if (request_message_queue.empty()) {
            continue;
        }

        std::tuple<unsigned int, MessageType, MessageContent> next_request = request_message_queue.front();
        request_message_queue.pop();

        unsigned int request_message_id = std::get<0>(next_request);
        MessageType request_type = std::get<1>(next_request);
        MessageContent request_content = std::get<2>(next_request);

        switch (request_type) {
            case MessageType::get_motor_position_counter: {
                int motor = std::get<int>(request_content);
                int position = get_motor_position_counter(motor);
                std::tuple<int, int> response_content_tuple = std::make_tuple(motor, position);
                response_message_queue.push(std::make_tuple(request_message_id, response_content_tuple));
                break;
            }

            case MessageType::clear_motor_position_counter: {
                int motor = std::get<int>(request_content);
                clear_motor_position_counter(motor);
                break;
            }

            case MessageType::set_motor_power: {
                std::tuple<int, int> content_tuple = std::get<std::tuple<int, int>>(request_content);
                int motor = std::get<0>(content_tuple);
                int power = std::get<1>(content_tuple);
                set_motor_power(motor, power);
                break;
            }

            case MessageType::move_at_velocity: {
                std::tuple<int, int> content_tuple = std::get<std::tuple<int, int>>(request_content);
                int motor = std::get<0>(content_tuple);
                int velocity = std::get<1>(content_tuple);
                move_at_velocity(motor, velocity);
                break;
            }

            case MessageType::move_to_position: {
                std::tuple<int, int, int> content_tuple = std::get<std::tuple<int, int, int>>(request_content);
                int motor = std::get<0>(content_tuple);
                int speed = std::get<1>(content_tuple);
                int goal_ticks = std::get<2>(content_tuple);
                move_to_position(motor, speed, goal_ticks);
                break;
            }

            case MessageType::move_relative_position: {
                std::tuple<int, int, int> content_tuple = std::get<std::tuple<int, int, int>>(request_content);
                int motor = std::get<0>(content_tuple);
                int speed = std::get<1>(content_tuple);
                int delta_ticks = std::get<2>(content_tuple);
                move_relative_position(motor, speed, delta_ticks);
                break;
            }

            case MessageType::is_motor_done: {
                int motor = std::get<int>(request_content);
                bool is_done = (bool)get_motor_done(motor);
                std::tuple<int, bool> response_content_tuple = std::make_tuple(motor, is_done);
                response_message_queue.push(std::make_tuple(request_message_id, response_content_tuple));
                break;
            }
            
            case MessageType::freeze: {
                int motor = std::get<int>(request_content);
                freeze(motor);
                break;
            }

            case MessageType::is_servo_enabled: {
                int servo = std::get<int>(request_content);
                bool is_enabled = (bool)get_servo_enabled(servo);
                std::tuple<int, bool> response_content_tuple = std::make_tuple(servo, is_enabled);
                response_message_queue.push(std::make_tuple(request_message_id, response_content_tuple));
                break;
            }

            case MessageType::set_servo_enabled: {
                std::tuple<int, bool> content_tuple = std::get<std::tuple<int, bool>>(request_content);
                int port = std::get<0>(content_tuple);
                bool enabled = std::get<1>(content_tuple);
                set_servo_enabled(port, enabled);
                break;
            }

            case MessageType::set_all_servos_enabled: {
                bool enabled = std::get<bool>(request_content);
                if (enabled) {
                    enable_servos();
                } else {
                    disable_servos();
                }
                break;
            }

            case MessageType::set_servo_position: {
                std::tuple<int, int> content_tuple = std::get<std::tuple<int, int>>(request_content);
                int port = std::get<0>(content_tuple);
                int position = std::get<1>(content_tuple);
                set_servo_position(port, position);
                break;
            }

            case MessageType::get_analog: {
                int port = std::get<int>(request_content);
                int analog_value = analog(port);
                std::tuple<int, int> response_content_tuple = std::make_tuple(port, analog_value);
                response_message_queue.push(std::make_tuple(request_message_id, response_content_tuple));
                break;
            }

            case MessageType::get_digital: {
                int port = std::get<int>(request_content);
                int digital_value = get_digital_value(port);
                std::tuple<int, int> response_content_tuple = std::make_tuple(port, digital_value);
                response_message_queue.push(std::make_tuple(request_message_id, response_content_tuple));
                break;
            }

            case MessageType::set_digital: {
                std::tuple<int, int> content_tuple = std::get<std::tuple<int, int>>(request_content);
                int port = std::get<0>(content_tuple);
                int value = std::get<1>(content_tuple);
                set_digital_value(port, value);
                break;
            }

            case MessageType::change_digital_port_mode: {
                std::tuple<int, bool> content_tuple = std::get<std::tuple<int, bool>>(request_content);
                int port = std::get<0>(content_tuple);
                bool is_input = std::get<1>(content_tuple);
                set_digital_output(port, is_input);
                break;
            }

            case MessageType::get_gyro_x_raw: {
                response_message_queue.push(std::make_tuple(request_message_id, (int)gyro_x()));
                break;
            }

            case MessageType::get_gyro_y_raw: {
                response_message_queue.push(std::make_tuple(request_message_id, (int)gyro_y()));
                break;
            }

            case MessageType::get_gyro_z_raw: {
                response_message_queue.push(std::make_tuple(request_message_id, (int)gyro_z()));
                break;
            }

            case MessageType::get_pid_gains: {
                int motor = std::get<int>(request_content);
                short p, i, d, pd, id, dd;
                get_pid_gains(motor, &p, &i, &d, &pd, &id, &dd);
                std::tuple<int, short, short, short, short, short, short> response_content_tuple = std::make_tuple(motor, p, i, d, pd, id, dd);
                response_message_queue.push(std::make_tuple(request_message_id, response_content_tuple));
                break;
            }

            case MessageType::set_pid_gains: {
                std::tuple<int, short, short, short, short, short, short> content_tuple = std::get<std::tuple<int, short, short, short, short, short, short>>(request_content);
                int motor = std::get<0>(content_tuple);
                short p = std::get<1>(content_tuple);
                short i = std::get<2>(content_tuple);
                short d = std::get<3>(content_tuple);
                short pd = std::get<4>(content_tuple);
                short id = std::get<5>(content_tuple);
                short dd = std::get<6>(content_tuple);
                set_pid_gains(motor, p, i, d, pd, id, dd);
                break;
            }

            case MessageType::getpwm: {
                int motor = std::get<int>(request_content);
                int pwm = getpwm(motor);
                std::tuple<int, int> response_content_tuple = std::make_tuple(motor, pwm);
                response_message_queue.push(std::make_tuple(request_message_id, response_content_tuple));
                break;
            }

            case MessageType::setpwm: {
                std::tuple<int, int> content_tuple = std::get<std::tuple<int, int>>(request_content);
                int motor = std::get<0>(content_tuple);
                int pwm = std::get<1>(content_tuple);
                setpwm(motor, pwm);
                break;
            }
        }
    }
}

int send_message(MessageType message_type, MessageContent request_content) {
    Nano::MutexLock lock(message_mutex);
    unsigned int message_id = next_message_id;
    request_message_queue.push(std::make_tuple(message_id, message_type, request_content));
    next_message_id++;
    lock.unlock();
    return message_id;
}

MessageContent send_and_wait_for_response(MessageType message_type, MessageContent request_content, int wait_time_ms) {
    int message_id = send_message(message_type, request_content);

    while (true) {
        Nano::wait_for_milliseconds(wait_time_ms);
        Nano::MutexLock lock(message_mutex);

        if (response_message_queue.empty()) {
            continue;
        }

        if (message_id == std::get<0>(response_message_queue.front())) {
            MessageContent response = std::get<1>(response_message_queue.front());
            response_message_queue.pop();
            return response;
        }
    }
}

void Nano::start_nano() {
    if (!has_already_started_worker_thread) {
        worker_thread.start();
        has_already_started_worker_thread = true;
    }
}

void Nano::wait_for_milliseconds(int milliseconds) {
    msleep(milliseconds);
}

Nano::BaseRobot::BaseRobot() {
    Nano::start_nano();
}

void Nano::BaseRobot::wait_for_light(int light_sensor_pin) {
    unsigned int off_light_value_sum = 0;
    unsigned int off_light_value_count = 0;
    float off_light_value_average = 0;

    unsigned int on_light_value_sum = 0;
    unsigned int on_light_value_count = 0;
    float on_light_value_average = 0;

    // Calibrate on light value
    while (!b_button()) {
        int light_value = get_analog(light_sensor_pin);
        std::cout << "Please turn ON the light, and press the B button! Current light value: " << light_value << std::endl;
        on_light_value_sum += light_value;
        on_light_value_count++;
        Nano::wait_for_milliseconds(10);
    }

    on_light_value_average = (float)on_light_value_sum / on_light_value_count;

    std::cout << "\n\nOn light value average: " << on_light_value_average << "\n\n" << std::endl;
    Nano::wait_for_milliseconds(1000);

    // Calibrate off light value
    while (!b_button()) {
        int light_value = get_analog(light_sensor_pin);
        std::cout << "Please turn OFF the light, and press the B button! Current light value: " << light_value << std::endl;
        off_light_value_sum += light_value;
        off_light_value_count++;
        Nano::wait_for_milliseconds(10);
    }

    off_light_value_average = (float)off_light_value_sum / off_light_value_count;

    std::cout << "\n\nOn light value average: " << on_light_value_average << "\n\n" << std::endl;
    std::cout << "Off light value average: " << off_light_value_average << "\n\n" << std::endl;
    
    float threshold = (on_light_value_average + off_light_value_average) / 2;
    std::cout << "Calculated light threshold value: " << threshold << "\n\n" << std::endl;


    int light_value = get_analog(light_sensor_pin);
    if (on_light_value_average < off_light_value_average) {
        while (true) {
            // Wait until light value goes BELOW threshold
            std::cout << "Waiting for light value to go BELOW threshold. Threshold: " << threshold << ". Light value: " << light_value << std::endl;
            
            if (light_value < threshold) {
                break;
            }

            light_value = get_analog(light_sensor_pin);
            Nano::wait_for_milliseconds(10);
        }
    } else {
        while (true) {
            // Wait until light value goes ABOVE threshold
            int light_value = get_analog(light_sensor_pin);
            std::cout << "Waiting for light value to go ABOVE threshold. Threshold: " << threshold << ". Light value: " << light_value << std::endl;

            if (light_value > threshold) {
                break;
            }

            light_value = get_analog(light_sensor_pin);
            Nano::wait_for_milliseconds(10);
        }
    }

    std::cout << "Starting! Last light value: " << light_value << " and threshold " << threshold << std::endl;
}

int Nano::BaseRobot::get_motor_position_counter(int motor) {
    return std::get<int>(send_and_wait_for_response(MessageType::get_motor_position_counter, motor, 3));
}

void Nano::BaseRobot::clear_motor_position_counter(int motor) {
    send_message(MessageType::clear_motor_position_counter, motor);
}

void Nano::BaseRobot::set_motor_power(int motor, int power) {
    send_message(MessageType::set_motor_power, std::make_tuple(motor, power));
}

void Nano::BaseRobot::move_at_velocity(int motor, int velocity) {
    send_message(MessageType::move_at_velocity, std::make_tuple(motor, velocity));
}

void Nano::BaseRobot::move_to_position(int motor, int position, int goal_ticks) {
    send_message(MessageType::move_to_position, std::make_tuple(motor, position, goal_ticks));
}

void Nano::BaseRobot::move_relative_position(int motor, int speed, int delta_ticks) {
    send_message(MessageType::move_relative_position, std::make_tuple(motor, speed, delta_ticks));
}

bool Nano::BaseRobot::is_motor_done(int motor) {
    return std::get<1>(std::get<std::tuple<int, bool>>(send_and_wait_for_response(MessageType::is_motor_done, motor, 3)));
}

void Nano::BaseRobot::wait_until_motor_done(int motor) {
    while (!this->is_motor_done(motor)) {
        Nano::wait_for_milliseconds(10);
    }
}

void Nano::BaseRobot::freeze(int motor) {
    send_message(MessageType::freeze, motor);
}

void Nano::BaseRobot::freeze_all_motors() {
    this->freeze(0);
    this->freeze(1);
    this->freeze(2);
    this->freeze(3);
}

bool Nano::BaseRobot::is_servo_enabled(int port) {
    return std::get<1>(std::get<std::tuple<int, bool>>(send_and_wait_for_response(MessageType::is_servo_enabled, port, 3)));
}

void Nano::BaseRobot::set_servo_enabled(int port, bool enabled) {
    send_message(MessageType::set_servo_enabled, std::make_tuple(port, enabled));
}

void Nano::BaseRobot::set_all_servos_enabled(bool enabled) {
    send_message(MessageType::set_all_servos_enabled, enabled);
}

void Nano::BaseRobot::set_servo_position(int port, int position) {
    send_message(MessageType::set_servo_position, std::make_tuple(port, position));
}

int Nano::BaseRobot::get_analog(int port) {
    return std::get<1>(std::get<std::tuple<int, int>>(send_and_wait_for_response(MessageType::get_analog, port, 3)));
}

int Nano::BaseRobot::get_digital(int port) {
    return std::get<1>(std::get<std::tuple<int, int>>(send_and_wait_for_response(MessageType::get_digital, port, 3)));
}

void Nano::BaseRobot::set_digital(int port, int value) {
    send_message(MessageType::set_digital, std::make_tuple(port, value));
}

void Nano::BaseRobot::change_digital_port_mode(int port, bool is_input) {
    send_message(MessageType::change_digital_port_mode, std::make_tuple(port, is_input));
}

int Nano::BaseRobot::get_gyro_x() {
    return std::get<int>(send_and_wait_for_response(MessageType::get_gyro_x_raw, 0, 3));
}

int Nano::BaseRobot::get_gyro_y() {
    return std::get<int>(send_and_wait_for_response(MessageType::get_gyro_y_raw, 0, 3));
}

int Nano::BaseRobot::get_gyro_z() {
    return std::get<int>(send_and_wait_for_response(MessageType::get_gyro_z_raw, 0, 3));
}

void Nano::BaseRobot::set_pid_gains(int motor, short p, short i, short d, short pd, short id, short dd) {
    send_message(MessageType::set_pid_gains, std::make_tuple(motor, p, i, d, pd, id, dd));
}

void Nano::BaseRobot::get_pid_gains(int motor, short& p, short& i, short& d, short& pd, short& id, short& dd) {
    std::tuple<int, short, short, short, short, short, short> response = std::get<std::tuple<int, short, short, short, short, short, short>>(send_and_wait_for_response(MessageType::get_pid_gains, motor, 3));
    p = std::get<1>(response);
    i = std::get<2>(response);
    d = std::get<3>(response);
    pd = std::get<4>(response);
    id = std::get<5>(response);
    dd = std::get<6>(response);
}

int Nano::BaseRobot::getpwm(int motor) {
    return std::get<1>(std::get<std::tuple<int, int>>(send_and_wait_for_response(MessageType::getpwm, motor, 3)));
}

void Nano::BaseRobot::setpwm(int motor, int pwm) {
    send_message(MessageType::setpwm, std::make_tuple(motor, pwm));
}

// Mutex

Nano::Mutex::Mutex() : _m(mutex_create()) {}

Nano::Mutex::~Mutex() {
    mutex_destroy(this->_m);
}

Nano::MutexLock::MutexLock(Nano::Mutex& mutex) : _m(mutex) {
    mutex_lock(this->_m);
}

void Nano::MutexLock::unlock() {
    mutex_unlock(this->_m);
}

Nano::MutexLock::~MutexLock() {
    this->unlock();
}

// Thread
template<typename Func>
Nano::Thread<Func>::Thread(Func thread_function, bool start_automatically) 
    : _t(thread_create(thread_function)) {
    if (start_automatically) {
        this->start();
    }
}

template<typename Func>
Nano::Thread<Func>::~Thread() {
    this->stop();
}

template<typename Func>
void Nano::Thread<Func>::start() {
    thread_start(this->_t);
}

template<typename Func>
void Nano::Thread<Func>::wait_for_thread() {
    thread_wait(this->_t);
}

template<typename Func>
void Nano::Thread<Func>::stop() {
    thread_destroy(this->_t);
}