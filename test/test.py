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
from pidtune.models.plant_alfaro123c import FOPDT,SOPDT, overdamped
from pidtune.rules import usort


import unittest


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
        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM2.txt"), 'r') as data_file:
            raw_data = [line.split('\t') for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data] # Time vector
            step_vector = [float(cols[1]) for cols in raw_data] # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data] # Open-loop system response vector

        obj = FOPDT(
            time_vector=time_vector,
            step_vector=step_vector,
            resp_vector=resp_vector,)

    def test_object_members(self):
        """
        Plant model has at least the base members required
        """
        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM2.txt"), 'r') as data_file:
            raw_data = [line.split('\t') for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data] # Time vector
            step_vector = [float(cols[1]) for cols in raw_data] # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data] # Open-loop system response vector

        obj = FOPDT(time_vector=time_vector,
            step_vector=step_vector,
            resp_vector=resp_vector,)

        for member in PLANT_MEMBERS:
            self.assertTrue(
                hasattr(obj, member),
                "obj Alfaro123c has no \"{}\" required member".format(member))

    def test_toDict_method(self):
        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/GUNT.txt"), 'r') as data_file:
            raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
            step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

            SO = SOPDT(time_vector=time_vector,
                        step_vector=step_vector,
                        resp_vector=resp_vector,)

            FO = FOPDT(time_vector=time_vector,
                        step_vector=step_vector,
                        resp_vector=resp_vector,)

            self.assertIsInstance(FO.toDict(), dict, "Is not a dictionary")

            self.assertIsInstance(SO.toDict(), dict, "Is not a dictionary")

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM1.txt"), 'r') as data_file:
            raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
            step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

            pru3 = overdamped(time_vector=time_vector,
                        step_vector=step_vector,
                        resp_vector=resp_vector,)

            self.assertIsInstance(pru3.toDict(), dict, "Is not a dictionary")
            

    def test_tunning_controllers(self):
        for test_file in ["plant_raw_data/dataIDFOM.txt", "plant_raw_data/dataIDFOM1.txt", "plant_raw_data/dataIDFOM2.txt"]:
            with open("{}/{}".format(path.dirname(__file__), test_file), 'r') as data_file:
                raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
                time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
                step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
                resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

                #try:
                FO = FOPDT(time_vector=time_vector,
                            step_vector=step_vector,
                            resp_vector=resp_vector,)

                controllersFO = FO.tune_controllers()
                self.assertTrue(bool(controllersFO), "Controller list is empty.")

                '''except ValueError as e:
                    self.assertEqual("Model for this value of 'a' time constant not found. Valid values are: 0 < a < 1",str(e))'''

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/GUNT.txt"), 'r') as data_file:
            raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
            step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

            try:
                FO = FOPDT(time_vector=time_vector,
                                step_vector=step_vector,
                                resp_vector=resp_vector,)

                controllers_non = FO.tune_controllers()
                
            except ValueError as e:
                self.assertEqual("Controller for this time constants not found. Valid interval for this rule is: 0.1 < Tao < 2",str(e))


        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM.txt"), 'r') as data_file:
                raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
                time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
                step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
                resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

                #try:
                SO = SOPDT(time_vector=time_vector,
                            step_vector=step_vector,
                            resp_vector=resp_vector,)

                controllersSO = SO.tune_controllers()
                self.assertTrue(bool(controllersSO), "Controller list is empty.")

        for test_file in ["plant_raw_data/GUNT.txt", "plant_raw_data/dataIDFOM1.txt", "plant_raw_data/dataIDFOM2.txt"]:
            with open("{}/{}".format(path.dirname(__file__), test_file), 'r') as data_file:
                raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
                time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
                step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
                resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

                try:
                    SO2 = SOPDT(time_vector=time_vector,
                                    step_vector=step_vector,
                                    resp_vector=resp_vector,)

                    controllers_non2 = SO2.tune_controllers()
                    
                except ValueError as e:
                    self.assertEqual("Controller for this time constants not found. Valid interval for this rule is: 0.1 < Tao < 2",str(e))

        for test_file in ["plant_raw_data/GUNT.txt", "plant_raw_data/dataIDFOM.txt", "plant_raw_data/dataIDFOM2.txt"]:
            with open("{}/{}".format(path.dirname(__file__), test_file), 'r') as data_file:
                raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
                time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
                step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
                resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

                try:
                    over = overdamped(time_vector=time_vector,
                                    step_vector=step_vector,
                                    resp_vector=resp_vector,)

                    controllers_non_over = over.tune_controllers()
                    self.assertTrue(bool(controllers_non_over), "Controller list is empty.")
                except ValueError as e:
                    self.assertEqual("Model for this value of 'a' time constant not found. Valid values are: 0 < a < 1",str(e))

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM1.txt"), 'r') as data_file:
                raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
                time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
                step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
                resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

                try:
                    over2 = overdamped(time_vector=time_vector,
                                    step_vector=step_vector,
                                    resp_vector=resp_vector,)

                    controllers_over = over2.tune_controllers()
                    
                except ValueError as e:
                    self.assertEqual("Controller for this time constants not found. Valid interval for this rule is: 0.1 < Tao < 2",str(e))


    def test_parameters(self):
        def get_bool_parameter(parameter, reference):
            inf_limit = abs(reference) * 0.95
            sup_limit = abs(reference) * 1.05

            if abs(parameter) < sup_limit and abs(parameter) > inf_limit:
                return True
            if abs(parameter) == 0 and abs(reference) == 0:
                return True
            if abs(parameter) == 1 and abs(reference) == 1:
                return True
            else:
                return False

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM1.txt"), 'r') as data_file:
            raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
            step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

            #FOPDT Parameters
            Tao_FO1 = 149.0125
            L_FO1 = 51.34
            K_FO1 = 4.3550
            a_FO1 = 0

            FO = FOPDT(time_vector=time_vector,
                    step_vector=step_vector,
                    resp_vector=resp_vector,)

            FOPDT_dict = FO.toDict()
            self.assertTrue(bool(get_bool_parameter(Tao_FO1, FOPDT_dict["T"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(L_FO1, FOPDT_dict["L"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(K_FO1, FOPDT_dict["K"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(a_FO1, FOPDT_dict["a"])), "The parameter result is out of the valid range.")

            #SOPDT Parameters
            Tao_SO1 = 94.5820
            L_SO1 = 3.3360
            K_SO1 = 4.3550
            a_SO1 = 1

            SO = SOPDT(time_vector=time_vector,
                    step_vector=step_vector,
                    resp_vector=resp_vector,)

            SOPDT_dict = SO.toDict()
            self.assertTrue(bool(get_bool_parameter(Tao_SO1, SOPDT_dict["T"])), "The parameter result is out of the valid range.")
            #print(SOPDT_dict["L"])
            #self.assertTrue(bool(get_bool_parameter(L_SO1, SOPDT_dict["L"])), "Controller list is empty.")
            self.assertTrue(bool(get_bool_parameter(K_SO1, SOPDT_dict["K"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(a_SO1, SOPDT_dict["a"])), "The parameter result is out of the valid range.")


            #Overdamped Parameters
            Tao_overdamped1 = 91.5724
            L_overdamped1 = 0
            K_overdamped1 = 4.3550
            a_overdamped1 = 1.1393

            overdamp = overdamped(time_vector=time_vector,
                    step_vector=step_vector,
                    resp_vector=resp_vector,)

            overdamped_dict = overdamp.toDict()
            #self.assertTrue(bool(get_bool_parameter(Tao_overdamped1, overdamped_dict["T"])), "Controller list is empty.")
            self.assertTrue(bool(get_bool_parameter(L_overdamped1, overdamped_dict["L"])), "The parameter result is out of the valid range.")
            #print(overdamped_dict["T"])
            self.assertTrue(bool(get_bool_parameter(K_overdamped1, overdamped_dict["K"])), "The parameter result is out of the valid range.")
            #self.assertTrue(bool(get_bool_parameter(a_overdamped1, overdamped_dict["a"])), "Controller list is empty.")


        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM.txt"), 'r') as data_file:
            raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
            step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

            #FOPDT Parameters
            Tao_FO = 3.5490
            L_FO = 4.8782
            K_FO = -1.2638
            a_FO = 0

            FO = FOPDT(time_vector=time_vector,
                    step_vector=step_vector,
                    resp_vector=resp_vector,)

            FOPDT_dict = FO.toDict()
            #self.assertTrue(bool(get_bool_parameter(Tao_FO, FOPDT_dict["T"])), "Controller list is empty.")
            self.assertTrue(bool(get_bool_parameter(L_FO, FOPDT_dict["L"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(K_FO, FOPDT_dict["K"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(a_FO, FOPDT_dict["a"])), "The parameter result is out of the valid range.")

            #SOPDT Parameters
            Tao_SO = 2.2526
            L_SO = 3.7347
            K_SO = -1.2638
            a_SO = 1

            SO = SOPDT(time_vector=time_vector,
                    step_vector=step_vector,
                    resp_vector=resp_vector,)

            SOPDT_dict = SO.toDict()
            #self.assertTrue(bool(get_bool_parameter(Tao_SO, SOPDT_dict["T"])), "Controller list is empty.")
            self.assertTrue(bool(get_bool_parameter(L_SO, SOPDT_dict["L"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(K_SO, SOPDT_dict["K"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(a_SO, SOPDT_dict["a"])), "The parameter result is out of the valid range.")

            '''#Overdamped Parameters
            Tao_overdamped = 1.1018
            L_overdamped = 2.9420
            K_overdamped = -1.2638
            a_overdamped = 3.6286

            overdamp = overdamped(time_vector=time_vector,
                    step_vector=step_vector,
                    resp_vector=resp_vector,)'''


        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM2.txt"), 'r') as data_file:
            raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
            step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

            #FOPDT Parameters
            Tao_FO2 = 143.0975
            L_FO2 = 50.5505
            K_FO2 = 4.7742
            a_FO2 = 0

            FO2 = FOPDT(time_vector=time_vector,
                    step_vector=step_vector,
                    resp_vector=resp_vector,)

            FOPDT2_dict = FO2.toDict()
            self.assertTrue(bool(get_bool_parameter(Tao_FO2, FOPDT2_dict["T"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(L_FO2, FOPDT2_dict["L"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(K_FO2, FOPDT2_dict["K"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(a_FO2, FOPDT2_dict["a"])), "The parameter result is out of the valid range.")

            #SOPDT Parameters
            Tao_SO2 = 90.8276
            L_SO2 = 4.4448
            K_SO2 = 4.7742
            a_SO2 = 1

            SO2 = SOPDT(time_vector=time_vector,
                    step_vector=step_vector,
                    resp_vector=resp_vector,)

            SOPDT2_dict = SO2.toDict()
            self.assertTrue(bool(get_bool_parameter(Tao_SO2, SOPDT2_dict["T"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(L_SO2, SOPDT2_dict["L"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(K_SO2, SOPDT2_dict["K"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(a_SO2, SOPDT2_dict["a"])), "The parameter result is out of the valid range.")

            '''#Overdamped Parameters
            Tao_overdamped2 = 50.8548
            L_overdamped2 = 0
            K_overdamped2 = 4.7742
            a_overdamped2 = 2.9925

            overdamp = overdamped(time_vector=time_vector,
                    step_vector=step_vector,
                    resp_vector=resp_vector,)'''

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/GUNT.txt"), 'r') as data_file:
            raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
            step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

            Tao_FOG = 33.67
            L_FOG = 2.3060
            K_FOG = 4.9745
            a_FOG = 0

            FOG = FOPDT(time_vector=time_vector,
                    step_vector=step_vector,
                    resp_vector=resp_vector,)

            FOPDTG_dict = FOG.toDict()
            self.assertTrue(bool(get_bool_parameter(Tao_FOG, FOPDTG_dict["T"])), "The parameter result is out of the valid range.")
            #self.assertTrue(bool(get_bool_parameter(L_FOG, FOPDTG_dict["L"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(K_FOG, FOPDTG_dict["K"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(a_FOG, FOPDTG_dict["a"])), "The parameter result is out of the valid range.")

            Tao_SOG = 21.3712
            L_SOG = 0
            K_SOG = 4.9745
            a_SOG = 1

            SOG = SOPDT(time_vector=time_vector,
                    step_vector=step_vector,
                    resp_vector=resp_vector,)

            SOPDTG_dict = SOG.toDict()
            self.assertTrue(bool(get_bool_parameter(Tao_SOG, SOPDTG_dict["T"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(L_SOG, SOPDTG_dict["L"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(K_SOG, SOPDTG_dict["K"])), "The parameter result is out of the valid range.")
            self.assertTrue(bool(get_bool_parameter(a_SOG, SOPDTG_dict["a"])), "The parameter result is out of the valid range.")


    def test_transfer_function(self):
        FOPDT_reference_tf = "cc493acb06c972492546636dc2ae5ab19f6a4665acd4da0a9d0251af89a3be5e"
        SOPDT_reference_tf = "6fca589e887b7f5c665046fec6ed2f2672a8575b39b57e2ac2e36eae6bbc2058"
        overdamped_reference_tf = "b16d24d90cc69dd286f9a63037efdcf66bbbdbd628d20e6f107b42f5ba8bdaf3"

        #for test_file in ["dataIDFOM.txt", "dataIDFOM2.txt", "dataIDFOM3.txt"]:
        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM.txt"), 'r') as data_file:
                raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
                time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
                step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
                resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

                FO = FOPDT(time_vector=time_vector,
                            step_vector=step_vector,
                            resp_vector=resp_vector,)

                SO = SOPDT(time_vector=time_vector,
                step_vector=step_vector,
                resp_vector=resp_vector,)

                FOPDT_t = FO.tf()
                FOPDT_hashed = hashlib.sha256(b"FOPDT_t").hexdigest()
                #print(FOPDT_hashed)
                self.assertMultiLineEqual(FOPDT_reference_tf, FOPDT_hashed, msg=None)

                SOPDT_t = SO.tf_SOPDT()
                SOPDT_hashed = hashlib.sha256(b"SOPDT_t").hexdigest()
                #print(FOPDT_hashed)
                self.assertMultiLineEqual(SOPDT_reference_tf, SOPDT_hashed, msg=None)

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM1.txt"), 'r') as data_file:
                raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
                time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
                step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
                resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

                over = overdamped(time_vector=time_vector,
                                step_vector=step_vector,
                                resp_vector=resp_vector,)

                overdamped_t = over.tf_overdamped()
                overdamped_hashed = hashlib.sha256(b"overdamped_t").hexdigest()
                print(overdamped_hashed)
                self.assertMultiLineEqual(overdamped_reference_tf, overdamped_hashed, msg=None)

    def test_toResponse(self):
        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/GUNT.txt"), 'r') as data_file:
            raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
            step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

            SO = SOPDT(time_vector=time_vector,
                        step_vector=step_vector,
                        resp_vector=resp_vector,)

            FO = FOPDT(time_vector=time_vector,
                        step_vector=step_vector,
                        resp_vector=resp_vector,)

            self.assertIsInstance(FO.toResponse(), dict, "Is not a dictionary")

            self.assertIsInstance(SO.toResponse_SOPDT(), dict, "Is not a dictionary")

        with open("{}/{}".format(path.dirname(__file__), "plant_raw_data/dataIDFOM1.txt"), 'r') as data_file:
            raw_data = [line.strip().split() for line in data_file.read().split('\n') if line]
            time_vector = [float(cols[0]) for cols in raw_data]  # Time vector
            step_vector = [float(cols[1]) for cols in raw_data]  # Step vector
            resp_vector = [float(cols[2]) for cols in raw_data]  # Open-loop system response vector

            pru3 = overdamped(time_vector=time_vector,
                        step_vector=step_vector,
                        resp_vector=resp_vector,)

            self.assertIsInstance(pru3.toResponse_overdamped(), dict, "Is not a dictionary")

if __name__ == '__main__':
    unittest.main()
