/**
 * Comprehensive BLDC motor control example using magnetic sensor
 *
 * Using serial terminal user can send motor commands and configure the motor and FOC in real-time:
 * - configure PID controller constants
 * - change motion control loops
 * - monitor motor variabels
 * - set target values
 * - check all the configuration values
 *
 * To check the config value just enter the command letter.
 * For example: - to read velocity PI controller P gain run: P
 *              - to set velocity PI controller P gain  to 1.2 run: P1.2
 *
 * To change the target value just enter a number in the terminal:
 * For example: - to change the target value to -0.1453 enter: -0.1453
 *              - to get the current target value enter: V3
 *
 * List of commands:
 *  - P: velocity PID controller P gain
 *  - I: velocity PID controller I gain
 *  - D: velocity PID controller D gain
 *  - R: velocity PID controller voltage ramp
 *  - F: velocity Low pass filter time constant
 *  - K: angle P controller P gain
 *  - N: angle P controller velocity limit
 *  - L: system voltage limit
 *  - C: control loop
 *    - 0: voltage
 *    - 1: velocity
 *    - 2: angle
 *  - V: get motor variables
 *    - 0: currently set voltage
 *    - 1: current velocity
 *    - 2: current angle
 *    - 3: current target value
 *
 */


#include <Arduino.h>

static constexpr auto I2C_SCL_IO = (GPIO_NUM_18);
static constexpr auto I2C_SDA_IO = (GPIO_NUM_17);

static constexpr auto gpio_a_h = GPIO_NUM_47;
static constexpr auto gpio_a_l = GPIO_NUM_48;
static constexpr auto gpio_b_h = GPIO_NUM_21;
static constexpr auto gpio_b_l = GPIO_NUM_2;
static constexpr auto gpio_c_h = GPIO_NUM_14;
static constexpr auto gpio_c_l = GPIO_NUM_1;
static constexpr auto gpio_enable = -1; // connected to the VIO/~Stdby pin of TMC6300-BOB
static constexpr auto gpio_fault = GPIO_NUM_38; //


#include "SimpleFOC.h"
#include "sensors/MagneticSensorSPI.h"
#include "BLDCMotor.h"
#include "drivers/BLDCDriver6PWM.h"
#include "mt6701.hpp"

// SPI magnetic sensor instance
//MagneticSensorSPI sensor = MagneticSensorSPI(12, 14, 0x3FFF);

MT6701Sensor encoder = MT6701Sensor();



// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(1);
BLDCDriver6PWM driver = BLDCDriver6PWM(gpio_a_h, gpio_a_l, gpio_b_h, gpio_b_l, gpio_c_h, gpio_c_l);

// commander interface
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {

	// initialize encoder sensor hardware
	encoder.init();

	// link the motor to the sensor
	motor.linkSensor(&encoder);

	// driver config
	// power supply voltage [V]
	driver.voltage_power_supply = 5;
	driver.init();
	// link driver
	motor.linkDriver(&driver);

	// set control loop type to be used
	motor.controller = MotionControlType::torque;

	// use monitoring with serial for motor init
	// monitoring port
	Serial.begin(2000000);
	// comment out if not needed
	motor.useMonitoring(Serial);
//	motor.monitor_downsample = 0; // initially disable real-time monitoring

// BLDCMotor( int pp , float R)
// - pp            - pole pair number
// - R             - motor phase resistance
// - KV            - motor kv rating (rmp/v)
// - L             - motor phase inductance


	// initialise motor
	motor.init();
	// align encoder and start FOC
	motor.initFOC();

	// set the inital target value
	motor.target = 2;

	// define the motor id
	command.add('M', onMotor, "motor");

	// Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
	Serial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));

	_delay(1000);
}


void loop() {
	// iterative setting FOC phase voltage
	motor.loopFOC();

	// iterative function setting the outter loop target
	motor.move();

	// monitoring
	motor.monitor();
	// user communication
	command.run();
}
