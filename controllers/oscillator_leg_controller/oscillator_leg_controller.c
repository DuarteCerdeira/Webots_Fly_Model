/*
 * File:          oscillator_leg_controller.c
 * Date:          July 2021
 * Description:   Oscillator-based controller to move the webots Fly model
 * Author:        Duarte Cerdeira
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <webots/supervisor.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/touch_sensor.h>

#define TEST

#define TIME_STEP 64
#define ANGULAR_VELOCITY 0.5

#ifndef TEST
  #define SIMULATION_RUN_TIME 5.0
#endif

#define PI 3.14159265359

#define N_LEGS 6
#define N_LEG_JOINTS 6

#define SENSOR_FILE_PATH "aux_files\\sensor_values\\"
#define PARAMETERS_FILE_PATH "aux_files\\controller_parameters\\"
#define RESULTS_FILE_PATH "aux_files\\simulation_results\\"

// File to save touch sensor values
static FILE *sensor_output;

// File to fetch leg parameters
static FILE *parameters_input;

// File to save simulation results
static FILE *results_output;

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

// Claw touch sensors
static WbDeviceTag claw[N_LEGS];
static const char *claw_sensors[N_LEGS] = {
  "lf_claw", "lm_claw", "lr_claw",
  "rf_claw", "rm_claw", "rr_claw"
};

// Define the joint limits and phase lags for each joint

// Front legs
const double front_upper_limits[N_LEG_JOINTS] = {0.2, 1.2, 0.4, 2.2, -1.0, 0.2};
const double front_lower_limits[N_LEG_JOINTS] = {-0.2, 0.8, 0.0, 1.4, -2.2, 0.2};
const int front_phases[N_LEG_JOINTS]          = {0, 0, PI/2, PI, 0, 0};

// Middle legs
const double middle_upper_limits[N_LEG_JOINTS] = {0.0, 0.2, 0.4, 1.6, -1.2, 0.5};
const double middle_lower_limits[N_LEG_JOINTS] = {0.0, -0.4, 0.0, 1.2, -1.6, 0.5};
const int middle_phases[N_LEG_JOINTS]          = {0, 0, PI/2, 0, PI, 0};

// Rear legs
const double rear_upper_limits[N_LEG_JOINTS] = {0.2, -1.0, 0.0, 2.6, -0.8, 0.4};
const double rear_lower_limits[N_LEG_JOINTS] = {-0.6, -0.8, -0.6, 0.6, -2.2, 0.2};
const int rear_phases[N_LEG_JOINTS]          = {0, 0, PI, 0, PI, 0};

/*
 * Function:    amplitude()
 * Description: calculates the amplitude of the joint movement
 * Arguments:   double upper_limit - upper joint limit (radian)
 *              double lower_limit - lower joint limit (radian)
 * Returns:     amplitude of joint movement
 */
double amplitude(const double upper_limit, const double lower_limit)
{
  return (upper_limit - lower_limit) / 2;
}

/*
 * Function:    average()
 * Description: calculates the average position of the joint movement
 * Arguments:   double upper_limit - upper joint limit (radian)
 *              double lower_limit - lower joint limit (radian)
 * Returns:     average position of the joint
 */
double average(const double upper_limit, const double lower_limit)
{
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
double oscillator(const double upper_limit, const double lower_limit, double t, const double phase)
{
  return amplitude(upper_limit, lower_limit) * cos(t + phase) + average(upper_limit, lower_limit);
}

/*
 * Function:    actuate_motors()
 * Description: actuates the motors to follow an oscillating movement pattern according to the gait provided
 * Arguments:   double t - time variable
 *              double gait[] - phase lags that define the robot gait pattern
 * Returns:     None
 */
void actuate_motors(double t, double gait[])
{
  for (int i = 0; i < N_LEG_JOINTS; i++) {
    wb_motor_set_position(lf_leg[i], oscillator(front_upper_limits[i], front_lower_limits[i], ANGULAR_VELOCITY * t, front_phases[i]));
    wb_motor_set_position(rf_leg[i], oscillator(front_upper_limits[i], front_lower_limits[i], ANGULAR_VELOCITY * t, front_phases[i] + gait[0]));

    wb_motor_set_position(lm_leg[i], oscillator(middle_upper_limits[i], middle_lower_limits[i], ANGULAR_VELOCITY * t, middle_phases[i] + gait[1]));
    wb_motor_set_position(rm_leg[i], oscillator(middle_upper_limits[i], middle_lower_limits[i], ANGULAR_VELOCITY * t, middle_phases[i] + gait[2]));

    wb_motor_set_position(lr_leg[i], oscillator(rear_upper_limits[i], rear_lower_limits[i], ANGULAR_VELOCITY * t, rear_phases[i] + gait[3]));
    wb_motor_set_position(rr_leg[i], oscillator(rear_upper_limits[i], rear_lower_limits[i], ANGULAR_VELOCITY * t, rear_phases[i] + gait[4]));
  }

  return;
}

/*
 * Function:    write_sensor_values()
 * Description: writes the sensor values in a text file
 * Arguments:   None
 * Returns:     None
 */
void write_sensor_values(void)
{
  static double t_stance_step[N_LEGS];
  static bool step[N_LEGS];

  for (int i = 0; i < N_LEGS; i++) {
    if (wb_touch_sensor_get_value(claw[i])) {
      if (!step[i])
        t_stance_step[i] = 0;
      else t_stance_step[i] += TIME_STEP;
      fprintf(sensor_output, "| ++ ");
      step[i] = true;
    }
    else {
      if (step[i])
        t_stance_step[i] = 0;
      else t_stance_step[i] += TIME_STEP;
      fprintf(sensor_output, "|    ");
      step[i] = false;
    }
  } 
  fprintf(sensor_output, "| "); 
  for (int i = 0; i < N_LEGS; i++) {
    if (t_stance_step[i] == 0.0)
      fprintf(sensor_output, " ====== , ");
    else
      fprintf(sensor_output, "%.3lf ms, ", t_stance_step[i] / 1000);
  }
  fprintf(sensor_output, "\n");
  return;
}

/*
 * Function:    calculate_velocity()
 * Description: calculates the average forward velocity of the robot
 * Arguments:   double time - time passed since the beginning of the simulation
 *              double initial_pos - starting position of the robot
 *              double final_pos - final position of the robot
 * Returns:     average forward velocity of the robot
 */
double calculate_velocity(double time, const double initial_pos, const double final_pos)
{
  return (final_pos - initial_pos) / ((double) time / 1000);
}

/*
 * Function:    cleanup()
 * Description: performs the cleanup of the program and exits in case of failure
 * Arguments:   int status - determines the output of the program (EXIT_FAILURE / EXIT_SUCCESS)
 * Returns:     None
 */
void cleanup(int status)
{
  if (sensor_output && parameters_input && results_output) {
    fclose(sensor_output);
    fclose(parameters_input);
    fclose(results_output);
  }

  wb_robot_cleanup();

  if (status) {
    printf("Something went wrong. Shutting down...\n");
    exit(status);
  }
}
/*
 * Function:    make_file_name()
 * Description: generates the correct file name using a common file path and a unique identifier for each robot that uses this controller
 * Arguments:   char * file_path - common path for the files 
 *              char * identifier - unique identifier to distinguish files
 * Returns:     unique file name
 */
char *make_file_name(const char *file_path, const char *identifier)
{
  char *str = (char *) malloc(strlen(file_path) + strlen(identifier) + strlen(".txt") + 1);
  if(str == NULL)
    cleanup(EXIT_FAILURE);

  str = strcpy(str, file_path);
  str = strcat(str, identifier);
  str = strcat(str, ".txt");

  return str;
}

/* 
 *
 * Main function
 * 
 */
int main(int argc, char **argv) {
  wb_robot_init();

  // Get the robot position

  WbNodeRef fly_node = wb_supervisor_node_get_self();
  if (fly_node == NULL)
    cleanup(EXIT_FAILURE);
  WbFieldRef fly_translation = wb_supervisor_node_get_field(fly_node, "translation");

  // Open files

  const char *fly_name = wb_robot_get_name();
  char *parameters_file_name = make_file_name(PARAMETERS_FILE_PATH, fly_name);
  char *results_file_name = make_file_name(RESULTS_FILE_PATH, fly_name);
  char *sensor_file_name = make_file_name(SENSOR_FILE_PATH, fly_name);

  if (parameters_file_name == NULL || results_file_name == NULL || sensor_file_name == NULL) // Check if the memory was allocated correctly
    cleanup(EXIT_FAILURE);

  parameters_input = fopen(parameters_file_name, "r");
  results_output = fopen(results_file_name, "w");
  sensor_output = fopen(sensor_file_name, "w");

  if (sensor_output == NULL|| parameters_input == NULL || results_output == NULL) // Check if the files were open
    cleanup(EXIT_FAILURE);

  // Free the allocated memory

  free(parameters_file_name);
  free(results_file_name);
  free(sensor_file_name);

  // Get gait pattern

  double gait[N_LEGS - 1];

  for (int i = 0; i < N_LEGS - 1; i++) {
    if (fscanf(parameters_input, "%lf", &gait[i]) < 1) // Check if the values were read properly
      cleanup(EXIT_FAILURE);
    gait[i] *= (2*PI / 360.0);
  }

  // Get motors

  for (int i = 0; i < N_LEG_JOINTS; i++) {
    lf_leg[i] = wb_robot_get_device(lf_joints[i]);
    lm_leg[i] = wb_robot_get_device(lm_joints[i]);
    lr_leg[i] = wb_robot_get_device(lr_joints[i]);
    rf_leg[i] = wb_robot_get_device(rf_joints[i]);
    rm_leg[i] = wb_robot_get_device(rm_joints[i]);
    rr_leg[i] = wb_robot_get_device(rr_joints[i]);
  }
  
  // Get sensors
  
  for (int i = 0; i < N_LEGS; i++) {
    claw[i] = wb_robot_get_device(claw_sensors[i]);
    wb_touch_sensor_enable(claw[i], TIME_STEP);
  }

  // Main feedback loop
  // Read sensor values and actuate the motors according to the given gait pattern

  fprintf(sensor_output, "==========TEST RESULT==========\n");
  fprintf(sensor_output, "| LF | LM | LR | RF | RM | RR |\n");

  const double initial_position = wb_supervisor_field_get_sf_vec3f(fly_translation)[2];

  int time = 0;
  #ifdef TEST
    while(wb_robot_step(TIME_STEP) > -1) {
      time += TIME_STEP;
      write_sensor_values();
      actuate_motors(time, gait);
    }
  #else
    for (time = 0; time / 1000 < SIMULATION_RUN_TIME && wb_robot_step(TIME_STEP) > -1; time += TIME_STEP) {
      write_sensor_values();
      actuate_motors(time, gait);
    };
  #endif

  // Calculate the average velocity of the robot at the end of the simulation

  const double final_position = wb_supervisor_field_get_sf_vec3f(fly_translation)[2];

  double av_velocity = calculate_velocity(time, initial_position, final_position);
  fprintf(results_output, "%lf", av_velocity);

  // Cleanup code
  #ifndef TEST
    wb_supervisor_simulation_quit(EXIT_SUCCESS);
  #endif

  cleanup(EXIT_SUCCESS);
  
  return EXIT_SUCCESS;
}
