#!/usr/bin/python

import hashlib
from sys import exit
from pidtune import __version__ as vs

print("PIDtune version: {}".format(vs.__version__));

import control
from pidtune import utils

from pidtune.models import controller
from pidtune.models import plant
from pidtune.models import system
import unittest
from plant_alfaro123c import Alfaro123c


e_plant = plant.FractionalOrderModel(
    alpha=1.6,
    time_constant=1.1,
    proportional_constant=1.0,
    dead_time_constant=1.1
)


#print(e_plant)
tf=e_plant.tf()
#print(x)
tf_hashed = hashlib.sha256(b"tf").hexdigest()
#print(tf_hashed)

controllers = e_plant.tune_controllers()
_controllers = [c.toDict() for c in controllers ]
#print(_controllers)
sys_list = [
    system.ClosedLoop(
        controller = controller,
        plant = e_plant)
    for controller in controllers ]

for system in sys_list:
    response=system.step_response()

response_output = hashlib.sha256(b"response").hexdigest()

#print(response_output)
#List of controllers reference to verify the outputs
controllers_list = [{'ctype': 'PID', 'Ms': '1.4', 'n_kp': 0.19147715177000726, 'n_ti': 0.5731720773427056, 'n_td': 1.6505422617593668, 'kp': 0.19147715177000726, 'ti': 0.6083527188887472, 'td': 1.751850643592542}, 
 {'ctype': 'PID', 'Ms': '2.0', 'n_kp': 0.34738347359603283, 'n_ti': 0.8615208013585709, 'n_td': 1.4569572527272414, 'kp': 0.34738347359603283, 'ti': 0.9143999552726448, 'td': 1.5463836098061396}, 
 {'ctype': 'PI', 'Ms': '1.4', 'n_kp': 0.0525002768352075, 'n_ti': 0.353960968500874, 'n_td': 0, 'kp': 0.0525002768352075, 'ti': 0.37568668481952405, 'td': 0}, 
 {'ctype': 'PI', 'Ms': '2.0', 'n_kp': 0.16588542044657903, 'n_ti': 0.7033787726683416, 'n_td': 0, 'kp': 0.16588542044657903, 'ti': 0.7465513511146991, 'td': 0}]


#Plant parameter values reference to verify the outputs
alph = 1.6
T = 1.1
K = 1.0
L = 1.1
IAE = 0.0
reference_values = [alph, T, K, L, IAE]

#Encrypted word reference to verify the system response output 
#Hardcoded from the verified results
reponse_hash="a9f4b3d22a523fdada41c85c175425bcd15b32b4cd0f54d9433accd52d7195a1"

#Encrypted word reference to verify the transfer function 
#Hardcoded from the verified results
tf_reference="8541c2d149faea7b7e534fcd56f5d352c7ed5a620d81394c002a17f07e18a567"



#Module to test if the plant parameters are retrieved correctly
class Test_plant(unittest.TestCase):
    def test_is_correct(self):
        self.assertEqual(e_plant.alpha, reference_values[0])
        self.assertEqual(e_plant.T, reference_values[1])
        self.assertEqual(e_plant.K, reference_values[2])
        self.assertEqual(e_plant.L, reference_values[3])
        self.assertEqual(e_plant.IAE, reference_values[4])

#Module to test if the list of controllers is retrieved correctly
class Test_controllers(unittest.TestCase):
    def test_controllers_are_correct(self):
        self.assertListEqual(_controllers, controllers_list)

#Module to test the step response of the system
class Test_response(unittest.TestCase):
    def test_response_correct(self):
        self.assertMultiLineEqual(response_output, reponse_hash, msg=None)

class Test_transfer(unittest.TestCase):
    def test_response_correct(self):
        self.assertMultiLineEqual(tf_hashed, tf_reference, msg=None)
        
if __name__ == '__main__':
    unittest.main()




planta_Alfaro = Alfaro123c
planta_Alfaro.print()