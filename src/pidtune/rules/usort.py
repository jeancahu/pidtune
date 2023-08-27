import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import control as ctl
import scipy.signal as signal
from ..models.controller import Controller

class usort():
    def __init__(self
                 ):
        # Global variables for Time values
        self.Ti : float = 0,
        self.Td : float = 0,

        # Dictionary to store all the tunning tables
        self.Alfaro123c_rule = {
            "Regulatory": {
                "PI": {
                    "MS2":{
                        '0': [0.265, 0.603, -0.971],
                        '0.25': [0.077, 0.739, -0.663],
                        '0.5': [0.023, 0.821, -0.625],
                        '0.75': [-0.128, 1.035, -0.555],
                        '1': [-0.244, 1.226, -0.517],
                        '2DoF': [0.730, 0.302, 0.386],
                    },
                    "MS18":{
                        '0': [0.229, 0.537, -0.952],
                        '0.25': [0.037, 0.684, -0.626],
                        '0.5': [-0.056, 0.803, -0.561],
                        '0.75': [-0.160, 0.958, -0.516],
                        '1': [-0.289, 1.151, -0.472],
                        '2DoF': [0.658, 0.578, 0.372],
                    },
                    "MS16":{
                        '0': [0.175, 0.466, -0.911],
                        '0.25': [-0.009, 0.612, -0.578],
                        '0.5': [0.080, 0.702, -0.522],
                        '0.75': [-0.247, 0.913, -0.442],
                        '1': [-0.394, 1.112, -0.397],
                        '2DoF': [0.649, 0.898, 0.446],
                    },
                    "MS14":{
                        '0': [0.016, 0.476, -0.708, -1.382, 2.837, 0.211],
                        '0.25': [-0.053, 0.507, -0.513, 0.866, 0.790, 0.520],
                        '0.5': [-0.129, 0.600, -0.449, 1.674, 0.268, 1.062],
                        '0.75': [-0.292, 0.792, -0.368, 2.130, 0.112, 1.654],
                        '1': [-0.461, 0.997, -0.317, 2.476, 0.073, 1.955],
                        '2DoF': [0.811, 1.205, 0.608],
                    }
                },
                "PID":{
                    "MS2":{
                        '0': [0.235, 0.840, -0.919],
                        '0.25': [0.435, 0.551, -1.123],
                        '0.5': [0.454, 0.588, -1.211],
                        '0.75': [0.464, 0.677, -1.251],
                        '1': [-0.488, 0.767, -1.273],
                        '2DoF': [0.306, 0.416, 0.367],
                    },
                    "MS18":{
                        '0': [0.210, 0.745, -0.919],
                        '0.25': [0.380, 0.500, -1.108],
                        '0.5': [0.400, 0.526, -1.194],
                        '0.75': [0.410, 0.602, -1.234],
                        '1': [0.432, 0.679, -1.257],
                        '2DoF': [0.248, 0.571, 0.362],
                    },
                    "MS16":{
                        '0': [0.179, 0.626, -0.921],
                        '0.25': [0.311, 0.429, -1.083],
                        '0.5': [0.325, 0.456, -1.160],
                        '0.75': [0.333, 0.519, -1.193],
                        '1': [0.351, 0.584, -1.217],
                        '2DoF': [0.255, 0.727, 0.476],
                    },
                    "MS14":{ #c y b
                        '0': [0.155, 0.455, -0.939, -0.198, 1.291, 0.485, 0.004, 0.389, 0.869],
                        '0.25': [0.228, 0.336, -1.057, 0.095, 1.165, 0.517, 0.104, 0.414, 0.758],
                        '0.5': [0.041, 0.571, -0.725, 0.132, 1.263, 0.496, 0.095, 0.540, 0.566],
                        '0.75': [0.231, 0.418, -1.136, 0.235, 1.291, 0.521, 0.074, 0.647, 0.511],
                        '1': [0.114, 0.620, -0.932, 0.236, 1.424, 0.495, 0.033, 0.756, 0.452],
                        '2DoF': [0.383, 0.921, 0.612],
                    }
                }
            },
            "Servo":{
                "PI":{
                    "MS18":{
                        '0': [0.243, 0.509, -1.063],
                        '0.25': [0.094, 0.606, -0.706],
                        '0.5': [0.013, 0.703, -0.621],
                        '0.75': [-0.075, 0.837, -0.569],
                        '1': [-0.164, 0.986, -0.531],
                    },
                    "MS16":{
                        '0': [0.209, 0.417, -1.064],
                        '0.25': [0.057, 0.528, -0.667],
                        '0.5': [-0.010, 0.607, -0.584],
                        '0.75': [-0.130, 0.765, -0.506],
                        '1': [-0.220, 0.903, -0.468],
                    },
                    "MS14":{
                        '0': [0.164, 0.305, -1.066, 14.650, 8.450, 0.0, 15.740],
                        '0.25': [0.019, 0.420, -0.617, 0.107, 1.164, 0.377, 0.066],
                        '0.5': [-0.061, 0.509, -0.511, 0.309, 1.362, 0.359, 0.146],
                        '0.75': [-0.161, 0.636, -0.439, 0.594, 1.532, 0.371, 0.237],
                        '1': [-0.253, 0.762, -0.397, 0.0625, 1.778, 0.355, 0.209],
                    }
                },
                "PID":{
                    "MS2":{
                        '0': [0.377, 0.727, -1.041],
                        '0.25': [0.502, 0.518, -1.194],
                        '0.5': [0.518, 0.562, -1.290],
                        '0.75': [0.533, 0.653, -1.329],
                        '1': [0.572, 0.728, -1.363],
                    },
                    "MS18":{
                        '0': [0.335, 0.644, -1.040],
                        '0.25': [0.432, 0.476, -1.163],
                        '0.5': [0.435, 0.526, -1.239],
                        '0.75': [0.439, 0.617, -1.266],
                        '1': [0.482, 0.671, -1.315],
                    },
                    "MS16":{
                        '0': [0.282, 0.544, -1.038],
                        '0.25': [0.344, 0.423, -1.117],
                        '0.5': [0.327, 0.488, -1.155],
                        '0.75': [0.306, 0.589, -1.154],
                        '1': [0.482, 0.622, -1.221],
                    },
                    "MS14":{
                        '0': [0.214, 0.413, -1.036, 1687, 339.2, 39.86, 1299, -0.016, 0.333, 0.815],
                        '0.25': [0.234, 0.352, -1.042, 0.135, 1.355, 0.333, 0.007, 0.026, 0.403, 0.613],
                        '0.5': [0.184, 0.423, -1.011, 0.246, 1.608, 0.273, 0.003, -0.042, 0.571, 0.446],
                        '0.75': [0.118, 0.575, -0.956, 0.327, 1.896, 0.243, -0.006, 0.086, 0.684, 0.772],
                        '1': [0.147, 0.607, -1.015, 0.381, 2.234, 0.204, -0.015, -0.110, 0.772, 0.372],
                    }
                }
            }
        }

    # Function to calculate proportional gains
    def calculate_Kp(self, a0, a1, a2, Tao, dc_gain):

        return (a0 + a1*(Tao**a2))/abs(dc_gain)

    # Function to calculate Ti for regulatory control
    def calculate_Ti_regulatory(self, b0, b1, b2, Tao, time_constant):

        return (b0 + b1*(Tao**b2))*time_constant

    # Function to calculate Td for regulatory control
    def calculate_Td_regulatory(self, c0, c1, c2, Tao, time_constant):

        return (c0 + (c1*Tao**c2))*time_constant

    # Function to calculate Ti for servo control
    def calculate_Ti_servo(self, b0, b1, b2, b3, Tao, time_constant):

        return ((b0 + (b1*Tao) + b2*(Tao*Tao))/(b3+Tao))*time_constant

    # Function to calculate Td for servo control
    def calculate_Td_servo(self, c0, c1, c2, Tao, time_constant):

        return (c0 + c1*(Tao**c2))*time_constant

    # Function to calculate the interpolation when a constant is not in the table values
    def interpolate(self, a_limit_i, a_limit_f, Kp1, Kp2, a_constant): #Cambiar nombres de parametros
        x1, y1 = a_limit_i, Kp1
        x2, y2 = a_limit_f, Kp2
        m= (y2 - y1) / (x2 - x1)
        b = y1 - m*x1
        Kp_final = m*a_constant + b

        return Kp_final

    # Function to select the specific controller
    # Calculates the parameters in function of values received
    def interpolated_parameters(self, control_mode, controller_type, constant_a, ms_key, time_constant, Tao, dc_gain, limit_1, limit_2, DoF):
        #Store a constant initial values
        a_initial_values = self.Alfaro123c_rule[control_mode][controller_type][ms_key][str(limit_1)]
        #Store a constant final values
        a_final_values = self.Alfaro123c_rule[control_mode][controller_type][ms_key][str(limit_2)]
        #Store b constant initial values
        b_initial_values = self.Alfaro123c_rule[control_mode][controller_type]['MS14'][str(limit_1)]
        #Store b constant final values
        b_final_values = self.Alfaro123c_rule[control_mode][controller_type]['MS14'][str(limit_2)]

        if control_mode == 'Regulatory':
            d_values = self.Alfaro123c_rule[control_mode][controller_type]['MS14']['2DoF']
            d0 = d_values[0]
            d1 = d_values[1]
            d2 = d_values[2]

        # Get first limit of 'a' values
        a0i = a_initial_values[0]
        a1i = a_initial_values[1]
        a2i = a_initial_values[2]

        # Get second limit of 'a' values
        a0f = a_final_values[0]
        a1f = a_final_values[1]
        a2f = a_final_values[2]

        # Get both proportional gains
        Kp1 = usort.calculate_Kp(self, a0i, a1i, a2i, Tao, dc_gain)
        Kp2 = usort.calculate_Kp(self, a0f, a1f, a2f, Tao, dc_gain)

        # Get the final model proportional gain
        Kp = usort.interpolate(self, limit_1, limit_2, Kp1, Kp2, constant_a)

        if control_mode == 'Regulatory':
            # Get first limit of 'b' values:
            b0i = b_initial_values[3]
            b1i = b_initial_values[4]
            b2i = b_initial_values[5]

            # Get second limit of 'b' values:
            b0f = b_final_values[3]
            b1f = b_final_values[4]
            b2f = b_final_values[5]

            # Get both Integral Times
            Ti1 = usort.calculate_Ti_regulatory(self, b0i, b1i, b2i, Tao, time_constant)
            Ti2 = usort.calculate_Ti_regulatory(self, b0f, b1f, b2f, Tao, time_constant)

            # Get the model final Integral Time
            self.Ti = usort.interpolate(self, limit_1, limit_2, Ti1, Ti2, constant_a)

            if controller_type == 'PID'and ms_key == 'MS14' and constant_a < 0.25 and Tao < 0.4:
                raise ValueError(f"Controller for this values constants not found. Valid interval for this rule is: a > 0.25 and Tao > 0.4")

            elif controller_type == 'PID':
                initial_c_values = self.Alfaro123c_rule[control_mode][controller_type]['MS14'][str(limit_1)]
                final_c_values = self.Alfaro123c_rule[control_mode][controller_type]['MS14'][str(limit_2)]

                # Get first limit of 'c' values
                c0i = initial_c_values[6]
                c1i = initial_c_values[7]
                c2i = initial_c_values[8]

                # Get second limit of 'c values
                c0f = final_c_values[6]
                c1f = final_c_values[7]
                c2f = final_c_values[8]

                # Get both derivative times
                Td1 = usort.calculate_Td_regulatory(self, c0i, c1i, c2i, Tao, time_constant)
                Td2 = usort.calculate_Td_regulatory(self, c0f, c1f, c2f, Tao, time_constant)

                # Get the model final derivative time
                self.Td = usort.interpolate(self, limit_1, limit_2, Td1, Td2, constant_a)

            else:
                self.Td = 0

            self.Parameters = {
                'Integral Time'    :   self.Ti,      # Time vector
                'Derivative Time'   :   self.Td,
                'Proportional Gain'    :   Kp,       # Step vector
                'Beta'  : 1
            }

            # 2 degrees of freedom, includes Beta parameter
            if DoF == 2:
                self.Parameters['Beta'] = d0 + d1*(Tao**d2)
            #print(self.Parameters)

            cont = Controller(
                    ctype = controller_type,
                    Ms = ms_key,
                    kp = Kp,
                    ti = self.Ti,
                    td = self.Td,
                    n_kp = Kp*abs(dc_gain),
                    n_ti = self.Ti/Tao,
                    n_td = self.Td / Tao,
                    beta = self.Parameters['Beta'],
                    action = dc_gain/abs(dc_gain)
                )

        elif control_mode == 'Servo':
            #Get first limit of 'b' values:
            b0i = b_initial_values[3]
            b1i = b_initial_values[4]
            b2i = b_initial_values[5]
            b3i = b_final_values[6]

            # Get second limit of 'b' values:
            b0f = b_final_values[3]
            b1f = b_final_values[4]
            b2f = b_final_values[5]
            b3f = b_final_values[6]

            # Get both Integral Times
            Ti1 = usort.calculate_Ti_servo(self, b0i, b1i, b2i, b3i, Tao, time_constant)
            Ti2 = usort.calculate_Ti_servo(self, b0f, b1f, b2f, b3f, Tao, time_constant)

            # Get the model final Integral Time
            self.Ti = usort.interpolate(self, limit_1, limit_2, Ti1, Ti2, constant_a)

            if controller_type == 'PID':
                initial_c_values = self.Alfaro123c_rule[control_mode][controller_type]['MS14'][str(limit_1)]
                final_c_values = self.Alfaro123c_rule[control_mode][controller_type]['MS14'][str(limit_2)]

                # Get first limit of 'c' values
                c0i = initial_c_values[7]
                c1i = initial_c_values[8]
                c2i = initial_c_values[9]

                # Get second limit of 'c values
                c0f = final_c_values[7]
                c1f = final_c_values[8]
                c2f = final_c_values[9]

                # Get both derivative times
                Td1 = usort.calculate_Td_servo(self, c0i, c1i, c2i, Tao, time_constant)
                Td2 = usort.calculate_Td_servo(self, c0f, c1f, c2f, Tao, time_constant)

                # Get the model final derivative time
                self.Td = usort.interpolate(self, limit_1, limit_2, Td1, Td2, constant_a)
            else:
                self.Td = 0

            self.Parameters = {
                'Integral Time'    :   self.Ti,      # Time vector
                'Derivative Time'   :   self.Td,
                'Proportional Gain'    :   Kp,       # Step vector
                'Beta'  : 1
            }

        cont = Controller(
                ctype = controller_type,
                Ms = ms_key,
                kp = Kp,
                ti = self.Ti,
                td = self.Td,
                n_kp = Kp*abs(dc_gain),
                n_ti = self.Ti/Tao,
                n_td = self.Td / Tao,
                action = dc_gain/abs(dc_gain),
                beta = self.Parameters['Beta']
            )

        return cont

    def get_value(self, control_mode, controller_type, constant_a, ms_key, time_constant, dc_gain, dead_time, DoF):

        # Calculates normalized time
        Tao = dead_time / time_constant

        print(Tao)

        # If is in usort restriction interval, raise an error
        if Tao < 0.1000 or Tao > 2: # usort restriction
            raise ValueError(f"Controller for this time constants not found. Valid interval for this rule is: 0.1 < Tao < 2")

        # Check if the parameters are valid
        if control_mode not in self.Alfaro123c_rule:
            raise ValueError(f"Control type {control_mode} not found. Valid options are 'Servo' or 'Regulatory'.")

        # Check if the parameters are valid
        if controller_type not in self.Alfaro123c_rule[control_mode]:
            raise ValueError(f"Degree of freedom {controller_type} not found. Valid options are 'PI' or 'PID'.")

        if controller_type == "PI" and control_mode == "Servo" and ms_key == "MS2":
            raise ValueError(f"This is not a valid option for the rule.")

        # Starts calculating the parameters
        # Calculates PI or PID regulatory parameters
        if control_mode == 'Regulatory':
            if constant_a >= 0 and constant_a < 0.25:
                self.Parameters=usort.interpolated_parameters(self,control_mode, controller_type, constant_a, ms_key, time_constant,Tao, dc_gain, 0, 0.25, DoF)

            elif constant_a >= 0.25 and constant_a < 0.50:
                self.Parameters=usort.interpolated_parameters(self,control_mode, controller_type, constant_a, ms_key, time_constant,Tao, dc_gain, 0.25, 0.50, DoF)

            elif constant_a >= 0.50 and constant_a < 0.75:
                self.Parameters=usort.interpolated_parameters(self,control_mode, controller_type, constant_a, ms_key, time_constant,Tao, dc_gain, 0.50, 0.75, DoF)

            elif constant_a >= 0.75 and constant_a <= 1:
                self.Parameters=usort.interpolated_parameters(self,control_mode, controller_type, constant_a, ms_key, time_constant,Tao, dc_gain, 0.75, 1, DoF)

        # Repeats the process for servo control, but without the Regulatory PID Ms 1.4 restriction
        elif control_mode == 'Servo':
            if constant_a >= 0 and constant_a < 0.25:
                self.Parameters=usort.interpolated_parameters(self,control_mode, controller_type, constant_a, ms_key, time_constant,Tao, dc_gain, 0, 0.25, DoF)

            elif constant_a >= 0.25 and constant_a < 0.50:
                self.Parameters=usort.interpolated_parameters(self,control_mode, controller_type, constant_a, ms_key, time_constant,Tao, dc_gain, 0.25, 0.50, DoF)

            elif constant_a >= 0.50 and constant_a < 0.75:
                self.Parameters=usort.interpolated_parameters(self,control_mode, controller_type, constant_a, ms_key, time_constant,Tao, dc_gain, 0.5, 0.75, DoF)

            elif constant_a >= 0.75 and constant_a <= 1:
                self.Parameters=usort.interpolated_parameters(self,control_mode, controller_type, constant_a, ms_key, time_constant,Tao, dc_gain, 0.75, 1, DoF)

        #print(self.Parameters)
        #print(type(self.Parameters))
        return(self.Parameters)

