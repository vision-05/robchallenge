/*
 * File:          my_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
const double BASE_SPEED = 5.0f;

bool following = true;

double error = 0.0f;
double previous_error = 0.0f;
double sum_error = 0.0f;
double gradient_error = 0.0f;
double control_signal = 0.0f;

const double Kp = 0.015f;
const double Ki = 0.0001f;
const double Kd = 0.2f;

double left = 0;
double right = 0;

double front_distance;
double left_distance;
double right_distance;

WbDeviceTag motor_fl;
WbDeviceTag motor_fr;
WbDeviceTag motor_bl;
WbDeviceTag motor_br;


void move(double left, double right){
  if (left > 10){
    left = 10;
  } else if (left < -10){
    left = -10;
  }
  
  if (right > 10){
    right = 10;
  } else if (right < -10){
    right = -10;
  }

  wb_motor_set_velocity(motor_fl, left);
  wb_motor_set_velocity(motor_fr, right);
  wb_motor_set_velocity(motor_bl, left);
  wb_motor_set_velocity(motor_br, right);
}



int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   
   motor_fl = wb_robot_get_device("motor_fl");
   motor_fr = wb_robot_get_device("motor_fr");
   motor_bl = wb_robot_get_device("motor_bl");
   motor_br = wb_robot_get_device("motor_br");
   
   wb_motor_set_position(motor_fl, INFINITY);
   wb_motor_set_position(motor_fr, INFINITY);
   wb_motor_set_position(motor_bl, INFINITY);
   wb_motor_set_position(motor_br, INFINITY);
   
   wb_motor_set_velocity(motor_fl, 0.0);
   wb_motor_set_velocity(motor_fr, 0.0);
   wb_motor_set_velocity(motor_bl, 0.0);
   wb_motor_set_velocity(motor_br, 0.0);
   
   WbDeviceTag ds_front = wb_robot_get_device("ds_front");
   WbDeviceTag ds_left = wb_robot_get_device("ds_left");
   WbDeviceTag ds_right = wb_robot_get_device("ds_right");

   
   wb_distance_sensor_enable(ds_front, TIME_STEP);
   wb_distance_sensor_enable(ds_left, TIME_STEP);
   wb_distance_sensor_enable(ds_right, TIME_STEP);
   

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
       
    front_distance = wb_distance_sensor_get_value(ds_front);
    left_distance = wb_distance_sensor_get_value(ds_left);
    right_distance = wb_distance_sensor_get_value(ds_right);
  
    
    
    error = 500 - left_distance;
  
    gradient_error = error - previous_error;
  
    sum_error = sum_error + error;
  
    control_signal = floor(Kp * error + Ki * sum_error + Kd * gradient_error);
  
    previous_error = error;
  
    left = BASE_SPEED + control_signal;
    right = BASE_SPEED - control_signal;
    
    printf("%.4f %.4f\n", left, right);
  
    //avoiding head on collision
    if (front_distance < 1000){
        move(BASE_SPEED, -BASE_SPEED);

    } else {
      move(left, right);
    }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
