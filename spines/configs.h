#include "vulp/actuation/wiringpi/wiringpi_config.h"

#include <string>

#define PI4B_CHIP_FREQ_HZ 19200000
#define SERVO_DELAY 3

#define TS90A_CLK_DIV 192
#define TS90A_FREQ_HZ 50
#define TS90A_RANGE (int)((float)PI4B_CHIP_FREQ_HZ / TS90A_CLK_DIV / TS90A_FREQ_HZ)
#define TS90A_MIN_PW_MS 0.5
#define TS90A_MAX_PW_MS 2.5
#define TS90A_MAX_DEGREE 90
#define TS90A_MIN_DEGREE -90
#define TS90A_HALL false
#define TS90A_HALL_A -1
#define TS90A_HALL_B -1
#define TS90A_HALL_RESO 11 * 18.75

#define MG995_360_CLK_DIV 192
#define MG995_360_FREQ_HZ 50
#define MG995_360_RANGE (int)((float)PI4B_CHIP_FREQ_HZ / TS90A_CLK_DIV / TS90A_FREQ_HZ)
#define MG995_360_MIN_PW_MS 0.5
#define MG995_360_MAX_PW_MS 2.5
#define MG995_360_MAX_DEGREE 180
#define MG995_360_MIN_DEGREE -180
#define MG995_360_HALL false
#define MG995_360_HALL_A -1
#define MG995_360_HALL_B -1
#define MG995_360_HALL_RESO 11 * 18.75

#define TB6612FNG_CLK_DIV 1
#define TB6612FNG_FREQ_HZ 100000
#define TB6612FNG_RANGE (int)((float)PI4B_CHIP_FREQ_HZ / TB6612FNG_CLK_DIV / TB6612FNG_FREQ_HZ)
#define TB6612FNG_MIN_PW_MS 0.0
#define TB6612FNG_MAX_PW_MS 0.1
#define TB6612FNG_MAX_DEGREE 100
#define TB6612FNG_MIN_DEGREE 0
#define TB6612FNG_HALL true
#define TB6612FNG_HALL_RESO 11 * 18.75



using namespace std;
using namespace vulp::actuation::wiringpi;

ServoConfig* TS90A_CONFIG(int id, string name, int bus_id, int pin, bool hall, int hall_a_pin, int hall_b_pin, float hall_ppr){
return new ServoConfig(
                            id,
                            SERVO_MODE_POS,
                            name,
                            bus_id, 
                            pin, 
                            -1,
                            -1,
                            -1,
                            PI4B_CHIP_FREQ_HZ, 
                            TS90A_CLK_DIV, 
                            TS90A_RANGE, 
                            TS90A_MIN_PW_MS, 
                            TS90A_MAX_PW_MS, 
                            TS90A_MIN_DEGREE, 
                            TS90A_MAX_DEGREE,
                            TS90A_FREQ_HZ, 
                            hall,
                            hall_a_pin,
                            hall_b_pin,
                            hall_ppr);
}

ServoConfig* MG995_360_CONFIG(int id, string name, int bus_id, int pin, bool hall, int hall_a_pin, int hall_b_pin, float hall_ppr){
return new ServoConfig(        
                            id,
                            SERVO_MODE_VEL,
                            name,
                            bus_id, 
                            pin,   
                            -1,
                            -1,
                            -1,
                            PI4B_CHIP_FREQ_HZ, 
                            MG995_360_CLK_DIV, 
                            MG995_360_RANGE, 
                            MG995_360_MIN_PW_MS, 
                            MG995_360_MAX_PW_MS, 
                            MG995_360_MIN_DEGREE, 
                            MG995_360_MAX_DEGREE,
                            MG995_360_FREQ_HZ, 
                            hall,
                            hall_a_pin,
                            hall_b_pin,
                            hall_ppr);
}


ServoConfig* TB6612FNG_MOTOR_CONFIG(int id, string name, int bus_id, int pin, int ctl_a_pin, int ctl_b_pin, int stby_pin, bool hall, int hall_a_pin, int hall_b_pin, float hall_ppr){
return new ServoConfig(        
                            id,
                            SERVO_MODE_TOR,
                            name,
                            bus_id, 
                            pin,  
                            ctl_a_pin,
                            ctl_b_pin,
                            stby_pin,
                            PI4B_CHIP_FREQ_HZ, 
                            TB6612FNG_CLK_DIV, 
                            TB6612FNG_RANGE, 
                            TB6612FNG_MIN_PW_MS, 
                            TB6612FNG_MAX_PW_MS, 
                            TB6612FNG_MIN_DEGREE, 
                            TB6612FNG_MAX_DEGREE,
                            TB6612FNG_FREQ_HZ, 
                            hall,
                            hall_a_pin,
                            hall_b_pin,
                            hall_ppr);
}


WiringpiConfig PI4B_CONFIG(){
    return WiringpiConfig(TB6612FNG_CLK_DIV, TB6612FNG_RANGE, SERVO_DELAY);
}

WiringpiConfig LESSIE_MARK1(){
    auto piConfig = PI4B_CONFIG();
    auto leftWheel = TB6612FNG_MOTOR_CONFIG(0, "left_wheel", 0, 1, 10, 11, 6, true, 24, 25, TB6612FNG_HALL_RESO);
    spdlog::info("[{}] - min_pwm:{}, max_pwm:{}, range:{}", leftWheel->name_, std::to_string(leftWheel->min_pwm_), std::to_string(leftWheel->max_pwm_), std::to_string(leftWheel->range_));

    auto rightWheel = TB6612FNG_MOTOR_CONFIG(0, "right_wheel", 0, 23, 4, 5, 6, true, 21, 22, TB6612FNG_HALL_RESO);
    spdlog::info("[{}] - min_pwm:{}, max_pwm:{}, range:{}", rightWheel->name_, std::to_string(rightWheel->min_pwm_), std::to_string(rightWheel->max_pwm_), std::to_string(rightWheel->range_));

    piConfig.addServo(leftWheel);
    piConfig.addServo(rightWheel);
    return piConfig;
}
