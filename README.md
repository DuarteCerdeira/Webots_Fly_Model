# Webots_Fly_Model

This is a Reference Manual for running the optimizer algorithm for the Webots fly and amputated fly models.

## SYSTEM REQUIREMENTS

- Python 3.9
- Webots (v. R2021b)

## EXECUTING THE PROGRAM

To execute the python script open a command line in the working directory and write the following command:

> python optimizer.py

The script will run the optimization algorithm, which runs several instances of the webots simulation, and write the final results in the command line.

## ALTERING PARAMETERS

Inside the python script there are several simulation parameters that can be adjusted. To alter them open *optimizer.py* in a text editor and edit the values under the section **# ========= Algorithm parameters ========= #**.

### DEFAULT VALUES AND DESCRIPTION

- PARTICLE_SIZE = 3    Describes the model to be tested: 3 for the amputated model, 5 for the non-amputated model
- N_PARTICLES = 5      Number of flies in the simulation, to change this value more flies need to be manually added in the simulation world file
- LOWER_BOUND = 0      Lower bound for the phase difference between each leg and the reference
- UPPER_BOUND = 180    Upper bound for the phase difference between each leg and the reference
- MAX_EVALS = 500      Maximum number of evaluations