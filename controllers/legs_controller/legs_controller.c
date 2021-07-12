/*
 * File:          legs_controller.c
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

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  WbDeviceTag lf_joints[6];
  char *lf_joints_names[6] = {
    "lf_tc_pro/re" , "lf_tc_rot" , "lf_tc_abd/add" ,
    "lf_ct_flex/ext" , "lf_ft_flex/ext" , "lf_tt_flex/ext"
  };
  
  WbDeviceTag lm_joints[6];
  char *lm_joints_names[6] = {
    "lm_tc_pro/re" , "lm_tc_rot" , "lm_tc_abd/add" ,
    "lm_ct_flex/ext" , "lm_ft_flex/ext" , "lm_tt_flex/ext"
  };
  
  WbDeviceTag lr_joints[6];
  char *lr_joints_names[6] = {
    "lr_tc_pro/re" , "lr_tc_rot" , "lr_tc_abd/add" ,
    "lr_ct_flex/ext" , "lr_ft_flex/ext" , "lr_tt_flex/ext"
  };
  
  WbDeviceTag rf_joints[6];
  char *rf_joints_names[6] = {
    "rf_tc_pro/re" , "rf_tc_rot" , "rf_tc_abd/add" ,
    "rf_ct_flex/ext" , "rf_ft_flex/ext" , "rf_tt_flex/ext"
  };
  
  WbDeviceTag rm_joints[6];
  char *rm_joints_names[6] = {
    "rm_tc_pro/re" , "rm_tc_rot" , "rm_tc_abd/add" ,
    "rm_ct_flex/ext" , "rm_ft_flex/ext" , "rm_tt_flex/ext"
  };
  
  WbDeviceTag rr_joints[6];
  char *rr_joints_names[6] = {
    "rr_tc_pro/re" , "rr_tc_rot" , "rr_tc_abd/add" ,
    "rr_ct_flex/ext" , "rr_ft_flex/ext" , "rr_tt_flex/ext"
  };
  
  for (int i = 0; i < 6; i++) {
    lf_joints[i] = wb_robot_get_device(lf_joints_names[i]);
    wb_motor_set_velocity(lf_joints[i], 10.0);
    lm_joints[i] = wb_robot_get_device(lm_joints_names[i]);
    wb_motor_set_velocity(lm_joints[i], 10.0);
    lr_joints[i] = wb_robot_get_device(lr_joints_names[i]);
    wb_motor_set_velocity(lr_joints[i], 10.0);
    rf_joints[i] = wb_robot_get_device(rf_joints_names[i]);
    wb_motor_set_velocity(rf_joints[i], 10.0);
    rm_joints[i] = wb_robot_get_device(rm_joints_names[i]);
    wb_motor_set_velocity(rm_joints[i], 10.0);
    rr_joints[i] = wb_robot_get_device(rr_joints_names[i]);
    wb_motor_set_velocity(rr_joints[i], 10.0);
  }
  
  wb_motor_set_position(lf_joints[0], 0.0);
  wb_motor_set_position(lf_joints[1], 0.6);
  wb_motor_set_position(lf_joints[2], 0.3);
  wb_motor_set_position(lf_joints[3], 1.5);
  wb_motor_set_position(lf_joints[4], -1.0);
  wb_motor_set_position(lf_joints[5], 0.4);
  
  wb_motor_set_position(lm_joints[0], 0.0);
  wb_motor_set_position(lm_joints[1], -0.1);
  wb_motor_set_position(lm_joints[2], 0.5);
  wb_motor_set_position(lm_joints[3], 1.2);
  wb_motor_set_position(lm_joints[4], -1.0);
  wb_motor_set_position(lm_joints[5], 0.5);
  
  wb_motor_set_position(lr_joints[0], 0.0);
  wb_motor_set_position(lr_joints[1], -0.6);
  wb_motor_set_position(lr_joints[2], 0.8);
  wb_motor_set_position(lr_joints[3], 0.8);
  wb_motor_set_position(lr_joints[4], -0.7);
  wb_motor_set_position(lr_joints[5], 0.2);
  
  wb_motor_set_position(rf_joints[0], 0.0);
  wb_motor_set_position(rf_joints[1], 0.6);
  wb_motor_set_position(rf_joints[2], 0.3);
  wb_motor_set_position(rf_joints[3], 1.5);
  wb_motor_set_position(rf_joints[4], -1.0);
  wb_motor_set_position(rf_joints[5], 0.4);

  wb_motor_set_position(rm_joints[0], 0.0);
  wb_motor_set_position(rm_joints[1], -0.1);
  wb_motor_set_position(rm_joints[2], 0.5);
  wb_motor_set_position(rm_joints[3], 1.2);
  wb_motor_set_position(rm_joints[4], -1.0);
  wb_motor_set_position(rm_joints[5], 0.5);
  
  wb_motor_set_position(rr_joints[0], 0.0);
  wb_motor_set_position(rr_joints[1], -0.6);
  wb_motor_set_position(rr_joints[2], 0.8);
  wb_motor_set_position(rr_joints[3], 0.8);
  wb_motor_set_position(rr_joints[4], -0.7);
  wb_motor_set_position(rr_joints[5], 0.2);

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
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
