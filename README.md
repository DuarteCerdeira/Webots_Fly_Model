# Webots_Fly_Model

This is a Reference Manual for running the optimizer algorithm for the Webots fly and amputated fly models.

## SYSTEM REQUIREMENTS

- Python 3.9 and inspyred library
- Webots (v. R2021b)

## EXECUTING THE PROGRAM

To execute the python script open a command line in the working directory and write the following command:

`python optimizer.py`

The script will run the optimization algorithm, which runs several instances of the webots simulation, and write the final results in the command line.

## ALTERING PARAMETERS

Inside the python script there are several simulation parameters that can be adjusted. To alter them open *optimizer.py* in a text editor and edit the values under the section **# ========= Algorithm parameters ========= #**.

### DEFAULT VALUES AND DESCRIPTION

- PARTICLE_SIZE = 3    Describes the model to be tested: 3 for the amputated model, 5 for the non-amputated model
- N_PARTICLES = 5      Number of flies in the simulation, to change this value more flies need to be manually added in the simulation world file
- LOWER_BOUND = 0      Lower bound for the phase difference between each leg and the reference
- UPPER_BOUND = 180    Upper bound for the phase difference between each leg and the reference
- MAX_EVALS = 500      Maximum number of evaluations

## SELECTING SIMULATIONS

There are 2 possible scenarios that can be simulated, a swarm of 6-legged flies and a swarm of 4-legged flies. To change the scenario, open *optimizer.py* in a text editor and edit the values under **# ========= Scenario selection ========= #**:

- In the variable **ARGUMENTS** write *"worlds\\amputated_fly_swarm.wbt"* for 4-legged flies or *"worlds\\fly_swarm.wbt"* for 6-legged flies.

- In the variable **AUX_FILES_PATH** write *"controllers\\amputated_fly_controller_swarm\\aux_files\\"* for 4-legged flies  or *"controllers\\fly_controller_swarm\\aux_files\\"* for 6-legged flies.

The rest should remain unchanged.

## READING THE OUTPUT VALUES

Each individual in the simulation records data that can be analyzed to help comprehend the results. This data is stored in a text file under the *aux_files\simulation_results* directory in the respective controller folder. That is *controllers\amputated_fly_controller_swarm\aux_files\simulation_results* for the 4-legged flies and *controllers\fly_controller_swarm\aux_files\simulation_results* for the 6-legged flies.

Example of a file:

	|===Checker Plots===|        LF         |        LR         |        RF         |        RR         |
	| LF | LR | RF | RR |    t    |    l    |    t    |    l    |    t    |    l    |    t    |    l    |
	|    |    |    |    | 0.064 s |         | 0.064 s |         | 0.064 s |         | 0.064 s |         |
	| ++ | ++ | ++ | ++ | ======= | 0.889 m | ======= | 1.133 m | ======= | 0.947 m | ======= | 1.126 m |
	| ++ | ++ | ++ | ++ | 0.064 s |         | 0.064 s |         | 0.064 s |         | 0.064 s |         |
	| ++ | ++ | ++ | ++ | 0.128 s |         | 0.128 s |         | 0.128 s |         | 0.128 s |         |
	| ++ | ++ | ++ | ++ | 0.192 s |         | 0.192 s |         | 0.192 s |         | 0.192 s |         |
	| ++ | ++ | ++ | ++ | 0.256 s |         | 0.256 s |         | 0.256 s |         | 0.256 s |         |
	| ++ | ++ | ++ | ++ | 0.320 s |         | 0.320 s |         | 0.320 s |         | 0.320 s |         |
	| ++ | ++ | ++ | ++ | 0.384 s |         | 0.384 s |         | 0.384 s |         | 0.384 s |         |
	| ++ |    | ++ | ++ | 0.448 s |         | ======= |         | 0.448 s |         | 0.448 s |         |
	| ++ |    | ++ | ++ | 0.512 s |         | 0.064 s |         | 0.512 s |         | 0.512 s |         |
	| ++ |    | ++ | ++ | 0.576 s |         | 0.128 s |         | 0.576 s |         | 0.576 s |         |
	| ++ | ++ | ++ | ++ | 0.640 s |         | ======= | 0.033 m | 0.640 s |         | 0.640 s |         |
	|    | ++ | ++ | ++ | ======= |         | 0.064 s |         | 0.704 s |         | 0.704 s |         |
	| ++ | ++ | ++ | ++ | ======= | 0.042 m | 0.128 s |         | 0.768 s |         | 0.768 s |         |
	| ++ | ++ | ++ | ++ | 0.064 s |         | 0.192 s |         | 0.832 s |         | 0.832 s |         |
	
	Average Velocity:
	0.013405

In the first part there is a checker plot graph describing the movement of the particular individual, after that there is a column registering the swing/stance duration and the step length for each respective leg (LF - Left front, RR - Right Rear, etc...).

In the end of the file the average velocity for the time the simulation was run is registered.

## ANALYSING INDIVIDUAL FILES

There are additional scenarios to help visualize particular gait patterns in a solo fly. These are the *amputated_fly.wbt* and *fly.wbt* world files.

To run these files, first write the gait pattern to be visualized in the *robot.txt* file in the corresponding *aux_files\controller_parameters* folder. These should follow the following model:

> 180.0 180.0 0.0 0.0 180.0 (Tripod gait)

... where each number corresponds to the phase gap of the leg in relation to the left front leg (RF LM RM LR RR). To analyse a gait pattern that results from the simulation, copy one of the *flyX.txt* file contents into the *robot.txt* file.

After the *robot.txt* file has been set up, open the world file located in the *worlds* directory using Webots. 

For further info on using the Webots simulator, refer to the [Cyberbotics](https://cyberbotics.com) team official website. 
