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
    std::tuple<int, short, short, short, short, short, short>,
> MessageContent;
std::queue<std::tuple<MessageType, MessageContent>> request_message_queue;
unsigned int next_message_id = 0;
std::queue<MessageContent> response_message_queue;
unsigned int top_most_request_message_id;


// delete this
int get_motor_position_counter(int motor);
void clear_motor_position_counter(int motor);
void set_motor_power(int motor, int power);
void move_at_velocity(int, int);
void move_to_position(int, int, int);
void move_relative_position(int, int, int);
int get_motor_done(int);
void freeze(int motor);
bool get_servo_enabled(int port);
void set_servo_enabled(int port, int enabled);
void enable_servos();
void disable_servos();
void set_servo_position(int port, int position);
int analog(int port);
void set_digital_value(int port, int value);
int get_digital_value(int port);
void set_digital_value(int port, int value);
void set_digital_output(int port, int out);
signed short gyro_x();
signed short gyro_y();
signed short gyro_z();
void set_pid_gains 	( 	int  	motor,
		short  	p,
		short  	i,
		short  	d,
		short  	pd,
		short  	id,
		short  	dd 
	) 		;
void get_pid_gains 	( 	int  	motor,
		short *  	p,
		short *  	i,
		short *  	d,
		short *  	pd,
		short *  	id,
		short *  	dd 
	) 	;

int getpwm(int motor);
int setpwm(int motor, int pwm);
void msleep(int milliseconds);
int b_button 	( 		) 	;

void worker_thread_function() {
    while (true) {
        Nano::wait_for_milliseconds(2);
        Nano::MutexLock lock(message_mutex);

        if (request_message_queue.empty()) {
            Nano::wait_for_milliseconds(3);
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
                std::tuple<int> content_tuple = std::get<std::tuple<int>>(request_content);
                int motor = std::get<0>(content_tuple);
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

void Nano::BaseRobot::wait_until_motor_done(int motor) {

}