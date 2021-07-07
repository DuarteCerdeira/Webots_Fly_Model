/*
 * File:          leg_movement.c
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

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

#define PI 3.14159

enum State {stroke1, stroke2};

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  int elapsed = 0;
  int i;
  enum State gait_state = stroke1;
  
  WbDeviceTag legs[4];
  char legs_names[4][3] = {"l1", "l2", "l3", "l4"};
  
  for (i = 0; i < 4; i++) {
    legs[i] = wb_robot_get_device(legs_names[i]);
    wb_motor_set_velocity(legs[i], 1);
  }

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    elapsed++;
    gait_state = (elapsed / 25) % 2;
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
    if (gait_state == stroke2) {
      wb_motor_set_position(legs[0], -2*PI/12);
      wb_motor_set_position(legs[1], 2*PI/12);
      wb_motor_set_position(legs[2], -2*PI/12);
      wb_motor_set_position(legs[3], 2*PI/12);
    } else if (gait_state == stroke1) {
      wb_motor_set_position(legs[0], 2*PI/12);
      wb_motor_set_position(legs[1], -2*PI/12);
      wb_motor_set_position(legs[2], 2*PI/12);
      wb_motor_set_position(legs[3], -2*PI/12);
    }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
