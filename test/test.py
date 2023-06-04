#!/usr/bin/python

import hashlib
from sys import exit
from os import path

from pidtune import __version__ as vs

print("PIDtune version: {}".format(vs.__version__));

import control
from pidtune import utils

from pidtune.models import controller
from pidtune.models import plant
from pidtune.models import system as close_loop_system
import unittest
from plant_alfaro123c import Alfaro123c

PLANT_MEMBERS = [
    '__init__',
    'tf',
    'tune_controllers',
    'toDict',
    'toResponse',
    '__str__',
]

class Test_fractional_model(unittest.TestCase):
    """
    Test class for fractional order model
    """
    def test_create_instance(self):
        """
        FractionalOrderModel instance raise ValueError when there is not input parameters
        """
        self.assertRaises(ValueError, plant.FractionalOrderModel)

    def test_object_members(self):
        """
        Plant model has at least the base members required
        """
        obj = plant.FractionalOrderModel(
            alpha=1.6,
            time_constant=1.1,
            proportional_constant=1.0,
            dead_time_constant=1.1
        )

        for member in PLANT_MEMBERS:
            self.assertTrue(
                hasattr(obj, member),
                "obj FractionalOrderModel has no \"{}\" required member".format(member))

    def test_parameters(self):
        #Plant parameter values reference to verify the outputs
        alph = 1.6
        T = 1.1
        K = 1.0
        L = 1.1
        IAE = 0.0

        e_plant = plant.FractionalOrderModel(
            alpha=1.6,
            time_constant=1.1,
            proportional_constant=1.0,
            dead_time_constant=1.1
        )

        self.assertEqual(e_plant.alpha, alph)
        self.assertEqual(e_plant.T, T)
        self.assertEqual(e_plant.K, K)
        self.assertEqual(e_plant.L, L)
        self.assertEqual(e_plant.IAE, IAE)

    def test_controllers(self):

        #List of controllers reference to verify the outputs
        controllers_reference = [
            {'ctype': 'PID',
             'Ms': '1.4',
             'n_kp': 0.19147715177000726,
             'n_ti': 0.5731720773427056,
             'n_td': 1.6505422617593668,
             'kp': 0.19147715177000726,
             'ti': 0.6083527188887472,
             'td': 1.751850643592542},
            {'ctype': 'PID',
             'Ms': '2.0',
             'n_kp': 0.34738347359603283,
             'n_ti': 0.8615208013585709,
             'n_td': 1.4569572527272414,
             'kp': 0.34738347359603283,
             'ti': 0.9143999552726448,
             'td': 1.5463836098061396},
            {'ctype': 'PI',
             'Ms': '1.4',
             'n_kp': 0.0525002768352075,
             'n_ti': 0.353960968500874,
             'n_td': 0,
             'kp': 0.0525002768352075,
             'ti': 0.37568668481952405,
             'td': 0},
            {'ctype': 'PI',
             'Ms': '2.0',
             'n_kp': 0.16588542044657903,
             'n_ti': 0.7033787726683416,
             'n_td': 0,
             'kp': 0.16588542044657903,
             'ti': 0.7465513511146991,
             'td': 0}
        ]

        e_plant = plant.FractionalOrderModel(
            alpha=1.6,
            time_constant=1.1,
            proportional_constant=1.0,
            dead_time_constant=1.1
        )

        controllers = e_plant.tune_controllers()

        # Convert controllers to controllers dictionaries
        controllers = [c.toDict() for c in controllers ]

        self.assertListEqual(
            controllers_reference,
            controllers,
            msg="Reference controllers and computed differ"
        )

    def test_tf(self):
        # Resulting transfer function hash
        tf_reference="49a165661c6446112b82fc0fb18a91f5a68f399ef0bb64f1a7ccf2e78e567bcc"

        e_plant = plant.FractionalOrderModel(
            alpha=1.6,
            time_constant=1.1,
            proportional_constant=1.0,
            dead_time_constant=1.1
        )

        tf=e_plant.tf()
        tf_hashed = hashlib.sha256(str(tf).encode('ascii')).hexdigest()

        self.assertEqual(
            tf_reference,
            tf_hashed,
            msg="Reference Transfer Function and computed one do not match"
        )

    def test_closeloop_system_response(self):
        # Resulting hashed close-loop system response list
        hashed_sys_response_list_reference = \
            ['062e7e4b3a43dcf088b55d74da354073a6f043179337e4e48f6d0fcb523b298f',
             'e6db3f0489f1b8d3e7047a4029da2cb75cb734bf25cf2ec43439cb2f61f8f2b1',
             '857462f0c65da6b3006a762e710c30f2d75dd36da2cc97d8635f30e7ddcd58c9',
             '0ac6851e54f9142093b729b733d7072b04beac2e3111433f2b2ab2671250856c']

        e_plant = plant.FractionalOrderModel(
            alpha=1.6,
            time_constant=1.1,
            proportional_constant=1.0,
            dead_time_constant=1.1
        )

        controllers = e_plant.tune_controllers()
        sys_list = [
            close_loop_system.ClosedLoop(
                controller = controller,
                plant = e_plant)
            for controller in controllers ]

        hashed_sys_response_list = list()
        for system in sys_list:
            response=system.step_response()
            hashed_sys_response_list.append(
                hashlib.sha256(str(response).encode('ascii')).hexdigest()
            )

        self.assertListEqual(
            hashed_sys_response_list_reference,
            hashed_sys_response_list,
            msg="Reference hashed list and computed one do not match"
        )

    def test_GUNT_identification (self):

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/GUNT.txt"), 'r') as data_file:
            raw_data = [line.split('\t') for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data] # Time vector
            step_vector = [float(cols[1]) for cols in raw_data] # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data] # Open-loop system response vector
            # TODO

    def test_dataIDFOM_identification (self):

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM.txt"), 'r') as data_file:
            raw_data = [line.split('\t') for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data] # Time vector
            step_vector = [float(cols[1]) for cols in raw_data] # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data] # Open-loop system response vector
            # TODO

    def test_dataIDFOM1_identification (self):

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM1.txt"), 'r') as data_file:
            raw_data = [line.split('\t') for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data] # Time vector
            step_vector = [float(cols[1]) for cols in raw_data] # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data] # Open-loop system response vector
            # TODO

    def test_dataIDFOM2_identification (self):

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM2.txt"), 'r') as data_file:
            raw_data = [line.split('\t') for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data] # Time vector
            step_vector = [float(cols[1]) for cols in raw_data] # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data] # Open-loop system response vector
            # TODO

## Alfaro123c Testing
class Test_Alfaro123c(unittest.TestCase):
    """
    Alfaro123c plant model test class
    """

    def test_create_instance(self):
        """
        Alfaro123c instance raise ValueError when there is not input parameters
        """
        self.assertRaises(ValueError, Alfaro123c)

    def test_object_members(self):
        """
        Plant model has at least the base members required
        """
        obj = Alfaro123c()
        for member in PLANT_MEMBERS:
            self.assertTrue(
                hasattr(obj, member),
                "obj Alfaro123c has no \"{}\" required member".format(member))

    def test_GUNT_identification (self):

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/GUNT.txt"), 'r') as data_file:
            raw_data = [line.split('\t') for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data] # Time vector
            step_vector = [float(cols[1]) for cols in raw_data] # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data] # Open-loop system response vector

            obj = Alfaro123c(
                time_vector=time_vector,
                step_vector=step_vector,
                resp_vector=resp_vector,
            )

    def test_dataIDFOM_identification (self):

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM.txt"), 'r') as data_file:
            raw_data = [line.split('\t') for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data] # Time vector
            step_vector = [float(cols[1]) for cols in raw_data] # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data] # Open-loop system response vector

            obj = Alfaro123c(
                time_vector=time_vector,
                step_vector=step_vector,
                resp_vector=resp_vector,
            )

    def test_dataIDFOM1_identification (self):

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM1.txt"), 'r') as data_file:
            raw_data = [line.split('\t') for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data] # Time vector
            step_vector = [float(cols[1]) for cols in raw_data] # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data] # Open-loop system response vector

            obj = Alfaro123c(
                time_vector=time_vector,
                step_vector=step_vector,
                resp_vector=resp_vector,
            )


    def test_dataIDFOM2_identification (self):

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM2.txt"), 'r') as data_file:
            raw_data = [line.split('\t') for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data] # Time vector
            step_vector = [float(cols[1]) for cols in raw_data] # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data] # Open-loop system response vector

            obj = Alfaro123c(
                time_vector=time_vector,
                step_vector=step_vector,
                resp_vector=resp_vector,
            )

if __name__ == '__main__':
    unittest.main()
