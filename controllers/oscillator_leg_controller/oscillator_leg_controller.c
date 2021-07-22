/*
 * File:          oscillator_leg_controller.c
 * Date:          July 2021
 * Description:   Oscillator-based controller to move the webots Fly model
 * Author:        Duarte Cerdeira
 */

#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 64
#define ANGULAR_VELOCITY 0.2

#define PI 3.14159265359

#define N_LEG_JOINTS 6


// Initialize robot information

// Left front leg motors
static WbDeviceTag lf_leg[N_LEG_JOINTS];
static const char *lf_joints[N_LEG_JOINTS] = {
  "lf_tc_pro/re",   "lf_tc_rot",       "lf_tc_abd/add",
  "lf_ct_flex/ext", "lf_ft_flex/ext" , "lf_tt_flex/ext"
};

// Left middle leg motors
static WbDeviceTag lm_leg[N_LEG_JOINTS];
static const char *lm_joints[N_LEG_JOINTS] = {
  "lm_tc_pro/re",   "lm_tc_rot",      "lm_tc_abd/add" ,
  "lm_ct_flex/ext", "lm_ft_flex/ext", "lm_tt_flex/ext"
};

// Left rear leg motors
static WbDeviceTag lr_leg[N_LEG_JOINTS];
static const char *lr_joints[N_LEG_JOINTS] = {
  "lr_tc_pro/re" , "lr_tc_rot" , "lr_tc_abd/add" ,
  "lr_ct_flex/ext" , "lr_ft_flex/ext" , "lr_tt_flex/ext"
};

// Right front leg motors
static WbDeviceTag rf_leg[N_LEG_JOINTS];
static const char *rf_joints[N_LEG_JOINTS] = {
  "rf_tc_pro/re" , "rf_tc_rot" , "rf_tc_abd/add" ,
  "rf_ct_flex/ext" , "rf_ft_flex/ext" , "rf_tt_flex/ext"
};

// Right middle leg motors
static WbDeviceTag rm_leg[N_LEG_JOINTS];
static const char *rm_joints[N_LEG_JOINTS] = {
  "rm_tc_pro/re" , "rm_tc_rot" , "rm_tc_abd/add" ,
  "rm_ct_flex/ext" , "rm_ft_flex/ext" , "rm_tt_flex/ext"
};

// Right rear leg motors
static WbDeviceTag rr_leg[N_LEG_JOINTS];
static const char *rr_joints[N_LEG_JOINTS] = {
  "rr_tc_pro/re" , "rr_tc_rot" , "rr_tc_abd/add" ,
  "rr_ct_flex/ext" , "rr_ft_flex/ext" , "rr_tt_flex/ext"
};

/*
 * Function:    amp()
 * Description: Calculates the amplitude of the joint movement
 * Arguments:   double upper_limit - upper joint limit (radian)
 *              dobule lower_limit - lower joint limit (radian)
 * Returns:     amplitude of joint movement
 */
double amp(const double upper_limit, const double lower_limit) {
  return (upper_limit - lower_limit) / 2;
}

/*
 * Function:    average()
 * Description: Calculates the average position of the joint movement
 * Arguments:   double upper_limit - upper joint limit (radian)
 *              double lower_limit - lower joint limit (radian)
 * Returns:     average position of the joint
 */
double average(const double upper_limit, const double lower_limit) {
  return (lower_limit + upper_limit) / 2;
}

/*
 * Function:    oscillator()
 * Description: controls the movement of a joint
 * Arguments:   double upper_limit - upper joint limit (radian)
 *              double lower_limit - lower joint limit (radian)
 *              double phase       - phase lag of the joint
 *              double t           - time variable
 * Returns:     instant position of the joint 
 */
double oscillator(const double upper_limit, const double lower_limit, double t, const double phase) {
  return amp(upper_limit, lower_limit) * cos(t + phase) + average(upper_limit, lower_limit);
}

/* 
 *
 * Main function
 * 
 */
int main(int argc, char **argv) {
  wb_robot_init();

  // Define the joint limits and phase lags for each joint

  // Front legs
  const double front_upper_limits[N_LEG_JOINTS] = {0.2, 1.2, 0.4, 2.2, -1.0, 0.2};
  const double front_lower_limits[N_LEG_JOINTS] = {-0.2, 0.8, 0.0, 1.4, -2.2, 0.2};
  const int front_phases[N_LEG_JOINTS]          = {0, 0, PI/2, PI, 0, 0};

  // Middle legs
  const double middle_upper_limits[N_LEG_JOINTS] = {0.0, 0.2, 0.2, 1.6, -1.2, 0.5};
  const double middle_lower_limits[N_LEG_JOINTS] = {0.0, -0.4, 0.0, 1.4, -1.4, 0.5};
  const int middle_phases[N_LEG_JOINTS]          = {0, 0, PI/2, 0, PI, 0};

  // Rear legs
  const double rear_upper_limits[N_LEG_JOINTS] = {0.2, -1.0, 0.0, 2.6, -0.8, 0.4};
  const double rear_lower_limits[N_LEG_JOINTS] = {-0.8, -0.8, -0.6, 0.6, -2.2, 0.2};
  const int rear_phases[N_LEG_JOINTS]          = {0, 0, PI, 0, PI, 0};

  // Phase lag between each leg to simulate gait
  double gait[5] = {PI, PI, 0, 0, PI};

  // Get motors
  for (int i = 0; i < N_LEG_JOINTS; i++) {
    lf_leg[i] = wb_robot_get_device(lf_joints[i]);
    lm_leg[i] = wb_robot_get_device(lm_joints[i]);
    lr_leg[i] = wb_robot_get_device(lr_joints[i]);
    rf_leg[i] = wb_robot_get_device(rf_joints[i]);
    rm_leg[i] = wb_robot_get_device(rm_joints[i]);
    rr_leg[i] = wb_robot_get_device(rr_joints[i]);
  }

  int time = 0;

  // Feedback loop
  while (wb_robot_step(TIME_STEP) != -1) {
    time += TIME_STEP;

    for (int i = 0; i < N_LEG_JOINTS; i++) {
      wb_motor_set_position(lf_leg[i], oscillator(front_upper_limits[i], front_lower_limits[i], ANGULAR_VELOCITY * time, front_phases[i]));
      wb_motor_set_position(rf_leg[i], oscillator(front_upper_limits[i], front_lower_limits[i], ANGULAR_VELOCITY * time, front_phases[i] + gait[0]));

      wb_motor_set_position(lm_leg[i], oscillator(middle_upper_limits[i], middle_lower_limits[i], ANGULAR_VELOCITY * time, middle_phases[i] + gait[1]));
      wb_motor_set_position(rm_leg[i], oscillator(middle_upper_limits[i], middle_lower_limits[i], ANGULAR_VELOCITY * time, middle_phases[i] + gait[2]));

      wb_motor_set_position(lr_leg[i], oscillator(rear_upper_limits[i], rear_lower_limits[i], ANGULAR_VELOCITY * time, rear_phases[i] + gait[3]));
      wb_motor_set_position(rr_leg[i], oscillator(rear_upper_limits[i], rear_lower_limits[i], ANGULAR_VELOCITY * time, rear_phases[i] + gait[4]));
    }
  };

  wb_robot_cleanup();

  return 0;
}
