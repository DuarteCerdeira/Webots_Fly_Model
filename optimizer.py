#
# File:         optimizer.py
# Date:         July 2021
# Description:  Optimizer algorithm to compute most optimal gait pattern for a fly
#               using a custom-made Webots fly model
# Author:       Duarte Cerdeira
#

from os import write
from time import time
from random import Random
from subprocess import call, run

from inspyred import ec, swarm
from inspyred.ec import *
from inspyred.swarm import *

WEBOTS_APP = ["C:\\Program Files\\Webots\\msys64\\mingw64\\bin\\webots.exe"]
ARGUMENTS = [
    "worlds\\amputated_fly_test.wbt", 
    "--mode=fast"
]
WEBOTS_CALL = WEBOTS_APP + ARGUMENTS

AUX_FILES_PATH = "controllers\\amputated_oscillator_leg_controller\\aux_files\\"

PARAM_FILE_PATH = AUX_FILES_PATH + "controller_parameters\\"
RESULTS_FILE_PATH = AUX_FILES_PATH + "simulation_results\\"

# ========= Algorithm parameters ========= #

PARTICLE_SIZE = 3   # Size of the parameter vector
N_PARTICLES = 5     # Number of individuals in the population
LOWER_BOUND = 0     # Lower bound for each value in the parameter vector
UPPER_BOUND = 180   # Upper bound for each value in the parameter vector
MAX_EVALS = 500     # Maximum number of evaluations 

def generator(random, args):
    """ Generates candidate solutions to the problem
    
        Generates a list of size specified by "num_inputs" (default 5)
        of values bound by "upper_bound" and "lower_bound" (default 0 - 360)

        The values represent phase lags therefore their minimum and maximum value are 0 and 360 respectively
    """
    particle = []
    size = args.get("num_inputs", 5)

    for i in range(size):
        particle.append(random.uniform(LOWER_BOUND, UPPER_BOUND))

    return particle

def evaluate(candidates, args):
    """ Evaluates the candidate solutions
    
        Evaluates the candidates given according to their resulting average
        speed simulated in Webots
    """
    fitness = []
    size = args.get("num_particles", 3)

    for i in range(size):
        params_file = open(PARAM_FILE_PATH + "fly{}".format(i) + ".txt", "w")
        for x in candidates[i]:
            params_file.write("{} ".format(x))
        params_file.close()

    run(WEBOTS_CALL)

    for i in range(size):
        results_file = open(RESULTS_FILE_PATH + "fly{}".format(i) + ".txt", "r")
        file_contents = results_file.read().split("\nAverage Velocity:\n")
        print(float(file_contents[1]))
        fitness.append(float(file_contents[1]))

    return fitness

rand = Random()
rand.seed(int(time()))

ea = swarm.PSO(rand)
ea.terminator = terminators.evaluation_termination
ea.topology = topologies.ring_topology
final_pop = ea.evolve(generator         = generator,
                      evaluator         = evaluate,
                      pop_size          = N_PARTICLES,
                      maximize          = True,
                      bounder           = Bounder(LOWER_BOUND, UPPER_BOUND),
                      num_particles     = N_PARTICLES,
                      num_inputs        = PARTICLE_SIZE,
                      max_evaluations   = MAX_EVALS,
                      inertia           = 0.7,
                      cognitive_rate    = 1.47,
                      social_rate       = 1.47)

best = max(final_pop)

print('Best Solution:\n{}'.format(best))