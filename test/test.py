#!/usr/bin/python

from sys import exit
from pidtune import __version__ as vs

print("PIDtune version: {}".format(vs.__version__));

import control
from pidtune import utils

from pidtune.models import controller
from pidtune.models import plant
from pidtune.models import system


e_plant = plant.FractionalOrderModel(
    alpha=1.6,
    time_constant=1.1,
    proportional_constant=1.0,
    dead_time_constant=1.1
)


print(e_plant)
e_plant.tf()


controllers = e_plant.tune_controllers()
sys_list = [
    system.ClosedLoop(
        controller = controller,
        plant = e_plant)
    for controller in controllers ]

for system in sys_list:
    system.step_response()


exit(0)
