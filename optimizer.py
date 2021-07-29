from os import write
from time import time
from random import Random
from subprocess import call, run

from inspyred import ec, swarm
from inspyred.ec import *
from inspyred.swarm import *

WEBOTS_APP = ["C:\\Program Files\\Webots\\msys64\\mingw64\\bin\\webots.exe"]
ARGUMENTS = [
    "worlds\\fly_test.wbt", 
    "--mode=realtime"
]

AUX_FILES_PATH = "controllers\\oscillator_leg_controller\\aux_files\\"

WEBOTS_CALL = WEBOTS_APP + ARGUMENTS
PARAM_FILE_NAME = AUX_FILES_PATH + "controller_parameters_"
RESULTS_FILE_NAME = AUX_FILES_PATH + "simulation_results_"

PARTICLE_SIZE = 5

def generator(random, args):
    """ Generates candidate solutions to the problem
    
        Generates a list of size specified by "num_inputs" (default 5)
        of values bound by "upper_bound" and "lower_bound" (default 0 - 360)

        The values represent phase lags therefore their minimum and maximum value are 0 and 360 respectively
    """
    particle = []
    size = args.get("num_inputs", 5)

    for i in range(size):
        particle.append(random.uniform(0, 180))

    return particle

def evaluate(candidates, args):
    """ Evaluates the candidate solutions
    
        Evaluates the candidates given according to their resulting average
        speed simulated in webots
    """
    fitness = []
    size = args.get("size", 5)

    for i in range(size):
        params_file = open(PARAM_FILE_NAME + "fly{}".format(i) + ".txt", "w")

        for x in candidates[i]:
            params_file.write("{} ".format(x))
        params_file.close()

    run(WEBOTS_CALL)

    for i in range(size):

        results_file = open(RESULTS_FILE_NAME + "fly{}".format(i) + ".txt", "r")
        fitness.append(float(results_file.read()))
        results_file.close()

    return fitness

rand = Random()
rand.seed(int(time()))

ea = swarm.PSO(rand)
ea.terminator = terminators.evaluation_termination
ea.topology = topologies.ring_topology
final_pop = ea.evolve(generator=generator,
                      evaluator=evaluate,
                      pop_size=5,
                      bounder=ec.Bounder(0, 180),
                      maximize=True,
                      max_evaluations=200,
                      num_inputs=5,
                      size=5)

best = max(final_pop)

print('Best Solution:\n{}'.format(best))