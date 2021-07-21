  /*
 * File:          oscillator_leg_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <math.h>

#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 64

#define PI 3.14159265359

#define N_LEG_JOINTS 6

static WbDeviceTag lf_leg[N_LEG_JOINTS];
static const char *lf_joints[N_LEG_JOINTS] = {
  "lf_tc_pro/re",   "lf_tc_rot",       "lf_tc_abd/add",
  "lf_ct_flex/ext", "lf_ft_flex/ext" , "lf_tt_flex/ext"
};

static WbDeviceTag lm_leg[N_LEG_JOINTS];
static const char *lm_joints[N_LEG_JOINTS] = {
  "lm_tc_pro/re",   "lm_tc_rot",      "lm_tc_abd/add" ,
  "lm_ct_flex/ext", "lm_ft_flex/ext", "lm_tt_flex/ext"
};

static WbDeviceTag lr_leg[N_LEG_JOINTS];
static const char *lr_joints[N_LEG_JOINTS] = {
  "lr_tc_pro/re" , "lr_tc_rot" , "lr_tc_abd/add" ,
  "lr_ct_flex/ext" , "lr_ft_flex/ext" , "lr_tt_flex/ext"
};

static WbDeviceTag rf_leg[N_LEG_JOINTS];
static const char *rf_joints[N_LEG_JOINTS] = {
  "rf_tc_pro/re" , "rf_tc_rot" , "rf_tc_abd/add" ,
  "rf_ct_flex/ext" , "rf_ft_flex/ext" , "rf_tt_flex/ext"
};

static WbDeviceTag rm_leg[N_LEG_JOINTS];
static const char *rm_joints[N_LEG_JOINTS] = {
  "rm_tc_pro/re" , "rm_tc_rot" , "rm_tc_abd/add" ,
  "rm_ct_flex/ext" , "rm_ft_flex/ext" , "rm_tt_flex/ext"
};

static WbDeviceTag rr_leg[N_LEG_JOINTS];
static const char *rr_joints[N_LEG_JOINTS] = {
  "rr_tc_pro/re" , "rr_tc_rot" , "rr_tc_abd/add" ,
  "rr_ct_flex/ext" , "rr_ft_flex/ext" , "rr_tt_flex/ext"
};

double amp(const double upper_limit, const double lower_limit) {
  return (upper_limit - lower_limit) / 2;
}

double average(const double upper_limit, const double lower_limit) {
  return (lower_limit + upper_limit) / 2;
}

double oscillator(const double upper_limit, const double lower_limit, const double phase, double x) {
  return amp(upper_limit, lower_limit) * cos(x + phase) + average(upper_limit, lower_limit);
}

int main(int argc, char **argv) {
  wb_robot_init();

  const double front_upper_limits[N_LEG_JOINTS] = {0.2, 1.2, 0.4, 2.2, -1.0, 0.2};
  const double front_lower_limits[N_LEG_JOINTS] = {-0.2, 0.8, 0.0, 1.4, -2.2, 0.2};
  const int front_phases[N_LEG_JOINTS]          = {0, 0, PI/2, PI, 0, 0};

  const double middle_upper_limits[N_LEG_JOINTS] = {0.0, 0.2, 0.2, 1.6, -1.2, 0.5};
  const double middle_lower_limits[N_LEG_JOINTS] = {0.0, -0.4, 0.0, 1.4, -1.4, 0.5};
  const int middle_phases[N_LEG_JOINTS]          = {0, 0, PI/2, 0, PI, 0};

  const double rear_upper_limits[N_LEG_JOINTS] = {0.2, -1.0, 0.0, 2.6, -0.8, 0.4};
  const double rear_lower_limits[N_LEG_JOINTS] = {-0.8, -0.8, -0.6, 0.6, -2.2, 0.2};
  const int rear_phases[N_LEG_JOINTS]          = {0, 0, PI, 0, PI, 0};

  for (int i = 0; i < N_LEG_JOINTS; i++) {
    lf_leg[i] = wb_robot_get_device(lf_joints[i]);
    lm_leg[i] = wb_robot_get_device(lm_joints[i]);
    lr_leg[i] = wb_robot_get_device(lr_joints[i]);
    rf_leg[i] = wb_robot_get_device(rf_joints[i]);
    rm_leg[i] = wb_robot_get_device(rm_joints[i]);
    rr_leg[i] = wb_robot_get_device(rr_joints[i]);
  }

  int step = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
    step++;
    if (step < 1.0 * 1000 / TIME_STEP)
      continue;
    for (int i = 0; i < N_LEG_JOINTS; i++) {
      wb_motor_set_position(lf_leg[i], oscillator(front_upper_limits[i], front_lower_limits[i], front_phases[i], 0.3 * (step * TIME_STEP)));
      wb_motor_set_position(rf_leg[i], oscillator(front_upper_limits[i], front_lower_limits[i], front_phases[i] + PI, 0.3 * (step * TIME_STEP)));

      wb_motor_set_position(lm_leg[i], oscillator(middle_upper_limits[i], middle_lower_limits[i], middle_phases[i] + PI, 0.3 * (step * TIME_STEP)));
      wb_motor_set_position(rm_leg[i], oscillator(middle_upper_limits[i], middle_lower_limits[i], middle_phases[i], 0.3 * (step * TIME_STEP)));

      wb_motor_set_position(lr_leg[i], oscillator(rear_upper_limits[i], rear_lower_limits[i], rear_phases[i], 0.3 * (step * TIME_STEP)));
      wb_motor_set_position(rr_leg[i], oscillator(rear_upper_limits[i], rear_lower_limits[i], rear_phases[i] + PI, 0.3 * (step * TIME_STEP)));
    }
  };

  wb_robot_cleanup();

  return 0;
}
