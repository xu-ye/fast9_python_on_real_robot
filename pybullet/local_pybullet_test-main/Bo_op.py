from bayes_opt import BayesianOptimization
from bayes_opt.logger import JSONLogger
from bayes_opt.event import Events
from bayes_opt.util import load_logs
from actuator_model import *

def black_box_function(x, y):
    """Function with unknown internals we wish to maximize.

    This is just serving as an example, for all intents and
    purposes think of the internals of this function, i.e.: the process
    which generates its output values, as unknown.
    """
    return -x ** 2 - (y - 1) ** 2 + 1


#pbounds = {'P_gain': (25, 60), 'D_gain': (0.5, 0.8),'friction':(0.0202,0.0204)}
pbounds = {'P_gain': (0.0001, 0.01), 'D_gain': (0.3, 0.4),'max_speed':(8.8,9.1)}
optimizer = BayesianOptimization(
    f=actuator_model_error,
    pbounds=pbounds,
    allow_duplicate_points=True,
    verbose=2, # verbose = 1 prints only when a maximum is observed, verbose = 0 is silent
    random_state=1,
)
#params={'P_gain': 33.9, 'D_gain': 0.6841,'friction':0.0203},
optimizer.probe(
    params={'P_gain': 0.01621, 'D_gain': 0.3618,'max_speed':8.92},
    lazy=True,
)
optimizer.maximize(
    init_points=1,
    n_iter=200,
)
print(optimizer.max)
logger = JSONLogger(path="./logs.json")
optimizer.subscribe(Events.OPTIMIZATION_STEP, logger)
optimizer.maximize(
    init_points=2,
    n_iter=10,
)
new_optimizer = BayesianOptimization(
    f=actuator_model_error,
    pbounds={'P_gain': (0, 4), 'D_gain': (0, 3)},
    verbose=2,
    random_state=7,
)
print(len(new_optimizer.space))
load_logs(new_optimizer, logs=["./logs.json"]);
print("New optimizer is now aware of {} points.".format(len(new_optimizer.space)))
#for i, res in enumerate(optimizer.res):
    #print("Iteration {}: \n\t{}".format(i, res))
new_optimizer.maximize(
    init_points=1,
    n_iter=10,
)