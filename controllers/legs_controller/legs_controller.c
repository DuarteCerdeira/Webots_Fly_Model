/*
 * File:          legs_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */


#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 64

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

void initial_position() {
  const double front_init_pos[N_LEG_JOINTS] = {0.0, 1.3, 0.1, 1.5, -1.0, 0.4};
  const double mid_init_pos[N_LEG_JOINTS]   = {0.0, 0.0, 0.5, 1.2, -1.0, 0.5};
  const double rear_init_pos[N_LEG_JOINTS]  = {0.0, -1.0, 0.8, 0.8, -0.7, 0.2};
  
  for (int i = 0; i < N_LEG_JOINTS; i++) {
    wb_motor_set_position(lf_leg[i], front_init_pos[i]);
    wb_motor_set_position(rf_leg[i], front_init_pos[i]);
    wb_motor_set_position(lm_leg[i], mid_init_pos[i]);
    wb_motor_set_position(rm_leg[i], mid_init_pos[i]);
    wb_motor_set_position(lr_leg[i], rear_init_pos[i]);
    wb_motor_set_position(rr_leg[i], rear_init_pos[i]);
  }
}

enum Side {left, right};

void front_swing(enum Side side) {
  const double target_pos[N_LEG_JOINTS] = {0.3, 1.4, -0.7, 1.5, -0.6, 0.4};

  for (int i = 0; i < N_LEG_JOINTS; i++) {
    if (side == left)
      wb_motor_set_position(lf_leg[i], target_pos[i]);
    else
      wb_motor_set_position(rf_leg[i], target_pos[i]);
  }
}

void front_stance(enum Side side) {
  const double target_pos[N_LEG_JOINTS] = {0.0, 1.2, 0.2, 3.0, -2.4, 0.4};

  for (int i = 0; i < N_LEG_JOINTS; i++) {
    if (side == left)
      wb_motor_set_position(lf_leg[i], target_pos[i]);
    else
      wb_motor_set_position(rf_leg[i], target_pos[i]);
  }
}

void mid_swing(enum Side side) {
  const double target_pos[N_LEG_JOINTS] = {0.5, 0.6, 0.3, 1.6, -1.6, 0.5};

  for (int i = 0; i < N_LEG_JOINTS; i++) {
    if (side == left)
      wb_motor_set_position(lm_leg[i], target_pos[i]);
    else
      wb_motor_set_position(rm_leg[i], target_pos[i]);
  }
}

void mid_stance(enum Side side) {
  const double target_pos[N_LEG_JOINTS] = {0.0, -0.6, 0.3, 1.9, -1.9, 0.5};

  for (int i = 0; i < N_LEG_JOINTS; i++) {
    if (side == left)
      wb_motor_set_position(lm_leg[i], target_pos[i]);
    else
      wb_motor_set_position(rm_leg[i], target_pos[i]);
  }
}

void rear_swing(enum Side side) {
  const double target_pos[N_LEG_JOINTS] = {0.5, -0.7, 0.7, 0.7, -0.9, 0.2};

  for (int i = 0; i < N_LEG_JOINTS; i++) {
    if (side == left)
      wb_motor_set_position(lr_leg[i], target_pos[i]);
    else
      wb_motor_set_position(rr_leg[i], target_pos[i]);
  }
}

void rear_stance(enum Side side) {
  const double target_pos[N_LEG_JOINTS] = {0.0, -1.3, 1.0, 1.9, -2.5, 0.2};

  for (int i = 0; i < N_LEG_JOINTS; i++){
    if (side == left)
      wb_motor_set_position(lr_leg[i], target_pos[i]);
    else
      wb_motor_set_position(rr_leg[i], target_pos[i]);
  }
  
}

int main(int argc, char **argv) {
  wb_robot_init(); 
  
  for (int i = 0; i < 6; i++) {
    lf_leg[i] = wb_robot_get_device(lf_joints[i]);
    lm_leg[i] = wb_robot_get_device(lm_joints[i]);
    lr_leg[i] = wb_robot_get_device(lr_joints[i]);
    rf_leg[i] = wb_robot_get_device(rf_joints[i]);
    rm_leg[i] = wb_robot_get_device(rm_joints[i]);
    rr_leg[i] = wb_robot_get_device(rr_joints[i]);
  }
  
  initial_position();

  int step = 0;
  while (wb_robot_step(TIME_STEP) != -1)
  {
    step++;
    if (step < 1.0 * 1000 / TIME_STEP)
      continue;
    
    else if (step * TIME_STEP / 1000 % 2 == 0) {
      front_swing(right);
      front_stance(left);
      mid_swing(left);
      mid_stance(right);
      rear_swing(left);
      rear_stance(right);
    }
    
    else if (step * TIME_STEP / 1000 % 2 == 1) {
      front_stance(right);
      front_swing(left);
      mid_stance(left);
      mid_swing(right);
      rear_stance(left);
      rear_swing(right);
    }
  }
  
  wb_robot_cleanup();

  return 0;
}
