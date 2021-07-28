from time import time
from random import Random
from subprocess import call, run

from inspyred import ec, swarm
from inspyred.ec import *
from inspyred.swarm import *

WEBOTS_APP = ["C:\\Program Files\\Webots\\msys64\\mingw64\\bin\\webots.exe"]
ARGUMENTS = [
    "worlds\\fly_test.wbt", 
    "--mode=fast"
]

AUX_FILES_PATH = "controllers\\oscillator_leg_controller\\aux_files\\"

WEBOTS_CALL = WEBOTS_APP + ARGUMENTS
PARAM_FILE_NAME = AUX_FILES_PATH + "controller_parameters.txt"
RESULTS_FILE_NAME = AUX_FILES_PATH + "simulation_results.txt"

PARTICLE_SIZE = 5

def generator(random, **args):
    particle = []
    size = PARTICLE_SIZE

    for i in range(size):
        particle.append(random.uniform(0, 360))

    return particle

def evaluate(candidates, **args):
    fitness = []

    for c in candidates:
        f_parameters = open(PARAM_FILE_NAME, "w")
        for x in c:
            f_parameters.write("{} ".format(x))
        f_parameters.close()

        run(WEBOTS_CALL)

        f_results = open(RESULTS_FILE_NAME, "r")
        fitness.append(float(f_results.read()))
        f_results.close()

    return fitness

rand = Random()
rand.seed(int(time()))

ea = swarm.PSO(rand)
ea.terminator = terminators.evaluation_termination
ea.topology = topologies.ring_topology
final_pop = ea.evolve(generator=generator,
                      evaluator=evaluate,
                      pop_size=1,
                      bounder=ec.Bounder(0, 360),
                      maximize=True,
                      max_evaluations=10)

best = max(final_pop)

print('Best Solution:\n{}'.format(best))