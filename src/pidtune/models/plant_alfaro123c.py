import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import control as ctl
import scipy.signal as signal
from io import StringIO
from scipy.signal import savgol_filter
from os import path
from ..rules.usort import usort
import hashlib


class Alfaro123c(): 
    '''
    Object: Alfaro123c
    Main class of Alfaro23c functionalities, that contains 
    the functions to process the input data and calculates
    the parameters for the respective models.    
    '''
    def __init__(self, 
                 time_vector : list = [],
                 step_vector : list = [],
                 resp_vector : list = [],

                 time_constant: float = 0,         # Main time constant (T)
                 dc_gain: float = 0, # Gain               (K)
                 dead_time: float = 0,    # Dead time          (L)
                 ):
        '''
        Function: __init__
        This function initialize the variables and vectors
        to store the input data and start executing the 
        ALfaro123c process to retrieve the possible models. 
        '''
        
         ## Identify the plant model
        if (time_constant or dc_gain or dead_time):
            try:
                self.T     = time_constant
                self.K     = dc_gain
                self.L     = dead_time
                self.IAE   = 0.0
                #self.controllers = self.tune_controllers()
            except Exception:
                raise ValueError("Plant model wrong input values")

        else:
            if not (len(time_vector) and len(step_vector) and len(resp_vector)):
                raise ValueError("Plant model wrong input values, no vectors or constants")

        # Variables to store response value at time points of interest
        self.response_at_25percent_time : float = 0
        self.response_at_50percent_time : float = 0
        self.response_at_75percent_time : float = 0

        # Simulation parameters
        self.t_simulation : list = [], # Time vector for simulation
        self.t_out_model_simulation : list = [], # Time vector for simulation
        self.u_simulation : list = [], # Input vector for simulation
        self.Yi : float = 0,
        self.Yf : float = 0,
        self.y_sys_simulation : list = [], # System output for simulation
        
        # Final model constants
        self.time_constant : float = 0, # Final model time constant
        self.dead_time : float = 0, # Final model dead time
        self.dc_gain : float = 0 # Final model DC Gain
        self.a_constant_time : float = 0

        # Create a dataframe with the input vector
        df = pd.DataFrame({'Tiempo': time_vector, 'Entrada': step_vector, 'Salida': resp_vector})

        # FOPDT values
        a_FOPDT = 0.9102
        b_FOPDT = 1.2620

        # SOPDT values
        a_SOPDT = 0.5776
        b_SOPDT = 1.5552

        # Calculates Input derivative
        filter_input = savgol_filter(df['Entrada'], window_length=51, polyorder=3, deriv=0)
        derivada_input = np.abs(np.gradient(filter_input, df['Tiempo']))
        df['Input_Derivative'] = derivada_input

        # Filtered plant response
        filter_output =  savgol_filter(df['Salida'], window_length=51, polyorder=3, deriv=0)

        # Calculates Output derivative
        df['Filtered'] = filter_output # Stores new vector with the output filtered
        derivada_output = np.abs(np.gradient(filter_output, df['Tiempo']))
        df['Output_Derivative'] = derivada_output
        

        # Calculates the 5% of the response change using the filtered output
        delta_output = np.subtract(np.max(filter_output, axis=0), np.min(filter_output, axis=0))
        outputChange_5percent = np.multiply(0.05, delta_output) # Gets the 'Delta Y' 5%

        # Gets the index of the max value
        max_value_output_index =  np.where(derivada_output == np.max(derivada_output, axis=0))[0][0]

        # Save the max and min value of the filtered response
        maxim_output = np.max(filter_output, axis=0)
        min_output = np.min(filter_output, axis=0)
        
        # Separates the output vector, before and after the maxim value
        before_transient_response = df.loc[:max_value_output_index]
        after_transient_response = df.loc[max_value_output_index:]

        # Define the vector with only the output values
        before_transient_response = before_transient_response['Salida']
        after_transient_response = after_transient_response['Salida']

        ### Verify if the signal is Increasing or decreasing
        if np.mean(before_transient_response) < np.mean(after_transient_response):
            ## If the mean of the first values is less than the mean of the last values
            ## the signal is Increasing

            # Values less than 5% of the output change magnitude
            valores_iniciales_estables = before_transient_response[ before_transient_response < (min_output + outputChange_5percent ) ]

            # Values greater than the 95% of the output change magnitude
            valores_finales_estables = after_transient_response[ after_transient_response > (maxim_output - outputChange_5percent ) ]

        else:
            # Decreasing signal

            # Values greater than the 95% of the output change magnitude
            valores_finales_estables = after_transient_response[ after_transient_response < (min_output + outputChange_5percent ) ]

            # Values less than 5% of the output change magnitude
            valores_iniciales_estables = before_transient_response[ before_transient_response > (maxim_output - outputChange_5percent ) ]

        # Output first and last values
        self.Yf = np.mean(valores_finales_estables)
        self.Yi = np.mean(valores_iniciales_estables)

        #Calculates change of the system response 
        SystemResponse_change = self.Yf - self.Yi
        SystemResponse_change = SystemResponse_change.item()

        #Input change#
        #Calculates the min and max point of the filtered input vector
        maxim_input = np.max(filter_input, axis=0)
        min_input = np.min(filter_input, axis=0)

        #Looks for the input max value index
        max_value_input_index = np.where(filter_input == np.max(filter_input, axis=0))[0][0]

        #Divides the input vector into, before and after the maximun value
        before_max_Input = df.loc[:max_value_input_index]
        after_max_Input = df.loc[max_value_input_index:]
        before_max_Input = before_max_Input['Entrada']
        after_max_Input = after_max_Input['Entrada']

        #Calculates the 10% of the input vector change
        delta_input = np.subtract(np.max(filter_input, axis=0), np.min(filter_input, axis=0))
        InputChange_10percent = np.multiply(0.1, delta_input) # Gets the 'Delta Y' 10%

        ### Verify if the signal is increasing or decreasing
        if np.mean(before_max_Input) < np.mean(after_max_Input):
            ## If the mean of the first values is less than the mean of the last values
            ## then the signal is Increasing

            # Values less than 10% of the output change magnitude
            valores_iniciales_estables_input = before_max_Input[ before_max_Input < (min_input + InputChange_10percent ) ]

            # Values greater than the 90% of the output change magnitude
            valores_finales_estables_input = after_max_Input[ after_max_Input > (maxim_input - InputChange_10percent ) ]

        else:
            # Decreasing signal

            # # Values of the input less than 90% of the output change magnitude
            valores_finales_estables_input = after_transient_response[ after_max_Input < (min_input + InputChange_10percent ) ]

            # Values of the input greater than the 10% of the output change magnitude
            valores_iniciales_estables_input = before_transient_response[ before_max_Input > (maxim_output - InputChange_10percent ) ]
        
        #Calculates the mean of the first and last values
        Uf = np.mean(valores_finales_estables_input)
        Ui = np.mean(valores_iniciales_estables_input)

        #Calculate input change
        systemStep_change = Uf - Ui

        #Calculate dc_gain
        self.dc_gain = SystemResponse_change/systemStep_change
        self.dc_gain =self.dc_gain

        #Uses the derivatives's max value to find the time point when input changes
        input_change = df.loc[[df['Input_Derivative'].idxmax()]]
        input_change_time = input_change['Tiempo'].max()
        
        # Absolute value of the system response change 
        SystemResponse_change_positivo = abs(SystemResponse_change)

        # Increasing case
        if np.mean(before_transient_response) < np.mean(after_transient_response):

            #FOPDT
            #Retrieves t25
            porcent25 = SystemResponse_change * 0.25    #Gets the 25% of the response magnitude
            p25 = self.Yi + porcent25                        #Add the 25% to the first value, to have the 25% in terms of the response
            puntos25 = df[df['Filtered'] < p25]           #Select all the values before the 25%
            self.response_at_25percent_time = puntos25['Tiempo'].max(numeric_only = True)  #Retrieves the last value before 25%
            self.response_at_25percent_time = self.response_at_25percent_time - input_change_time  #Substract the initial time to the time found in order to have the time to reaches the 25% of the response
            
            #Retrieves t75
            porcent75 = SystemResponse_change * 0.75     #Gets the 75% of the response magnitude
            p75 = self.Yi + porcent75         #Add the 75% to the first value, to have the 75% in terms of the response
            puntos75 = df[df['Filtered'] < p75]            #Select all the values before the 75%
            self.response_at_75percent_time = puntos75['Tiempo'].max()  #Retrieves the last value before 75%
            self.response_at_75percent_time = self.response_at_75percent_time - input_change_time #Substract the initial time to the time found in order to have the time to reaches the 75% of the response
            
            #Retrieves t50
            porcent50 = SystemResponse_change * 0.50        #Gets the 50% of the response magnitude
            p50 = self.Yi + porcent50            #Add the 50% to the first value, to have the 50% in terms of the response
            puntos50 = df[df['Filtered'] < p50]               #Select all the values before the 50%
            self.response_at_50percent_time = puntos50['Tiempo'].max()  #Retrieves the last value before 50%
            self.response_at_50percent_time = self.response_at_50percent_time - input_change_time #Substract the initial time to the time found in order to have the time to reaches the 50% of the response
        
        else:
            # Decreasing case 
            #Retrieves t25
            porcent25 = SystemResponse_change_positivo * 0.25 #Gets the 25% of the response magnitude
            p25 = self.Yi - porcent25                             #Add the 25% to the first value, to have the 25% in terms of the response
            puntos25 = df[df['Filtered'] > p25]                #Select all the values before the 25%
            self.response_at_25percent_time = puntos25['Tiempo'].max()#Retrieves the last value before 25%
            self.response_at_25percent_time = self.response_at_25percent_time - input_change_time #Substract the initial time to the time found in order to have the time to reaches the 25% of the response
            
            porcent75 = SystemResponse_change_positivo * 0.75 #Gets the 75% of the response magnitude
            p75 = self.Yi - porcent75            #Add the 75% to the first value, to have the 75% in terms of the response
            puntos75 = df[df['Filtered'] > p75]  #Select all the values before the 75%
            self.response_at_75percent_time = puntos75['Tiempo'].max()  #Retrieves the last value before 75%
            self.response_at_75percent_time = self.response_at_75percent_time - input_change_time  #Substract the initial time to the time found in order to have the time to reaches the 75% of the response

            #Retrieves t50
            porcent50 = SystemResponse_change_positivo * 0.50  #Gets the 50% of the response magnitude
            p50 = self.Yi - porcent50            #Add the 50% to the first value, to have the 50% in terms of the response
            puntos50 = df[df['Filtered'] > p50]        #Select all the values before the 50%
            self.response_at_50percent_time = puntos50['Tiempo'].max() #Retrieves the last value before 50%
            self.response_at_50percent_time = self.response_at_50percent_time - input_change_time  #Substract the initial time to the time found in order to have the time to reaches the 50% of the response

        # Calculate FOPDT parameters 
        self.FOPDT_time_constant = a_FOPDT*(self.response_at_75percent_time - self.response_at_25percent_time)
        self.FOPDT_dead_time = b_FOPDT*self.response_at_25percent_time + (1-b_FOPDT)*self.response_at_75percent_time
        if(self.FOPDT_dead_time.item() < 0):      #If FOPDT_dead_time is < 0, FOPDT_dead_time = 0
            self.FOPDT_dead_time = 0
        else:
            self.FOPDT_dead_time = self.FOPDT_dead_time.item()
    
        self.FOPDT_time_constant = self.FOPDT_time_constant.item()

        # Calculate SOPDT parameters 
        self.SOPDT_time_constant = a_SOPDT*(self.response_at_75percent_time - self.response_at_25percent_time)
        self.SOPDT_dead_time = b_SOPDT*self.response_at_25percent_time + (1-b_SOPDT)*self.response_at_75percent_time
        if(self.SOPDT_dead_time.item() < 0):      #If FOPDT_dead_time is < 0, FOPDT_dead_time = 0
            self.SOPDT_dead_time = 0
        else:
            self.SOPDT_dead_time = self.SOPDT_dead_time.item()
        
        self.SOPDT_time_constant = self.SOPDT_time_constant.item()




        # Simulation
        # Time vector to simulate the system
        self.t_simulation = df['Tiempo'].values

        # Input and output vector to simulate the system
        self.u_simulation = df['Entrada'].values

        # Input vector for the simulation
        # Looks for the diference between consecutive positions
        # Adds an input simulation vector "Cambio" that contains tha magnitude of the input changes
        df['Cambio'] = df['Entrada'].diff().fillna(0)

        # Finds the first non zero value
        first_non_zero = df['Cambio'].ne(0).idxmax()

        # Replace the non useful values by NaN
        df.loc[first_non_zero:, 'Cambio'] = np.where(df.loc[first_non_zero:, 'Cambio'] == 0, np.nan, df.loc[first_non_zero:, 'Cambio'])

        # Fill NaN values with the last valid value
        df['Cambio'].fillna(method='ffill', inplace=True)

        self.input_simulation = df['Cambio'].values

        # Output vector
        self.y_sys_simulation = df['Salida'].values

        #FOPDT Simulation
        # Define numerator and denominator to the transfer function without deadtime
        num = [self.dc_gain]  # Numerator
        den = [self.FOPDT_time_constant, 1]  # Denominator

        # Define transfer function
        tf_FOPDT = ctl.tf(num, den)
        
        # Simulate the FOPDT model response to the input vector
        self.t_out_model_simulation, self.youtFOPDT_simulation = ctl.forced_response(tf_FOPDT, self.t_simulation, self.input_simulation)

        # SOPDT parameters and vectors
        a_SOPDT = self.SOPDT_time_constant
        num_SOPDT = [self.dc_gain]
        den_SOPDT = [a_SOPDT*a_SOPDT, 2*a_SOPDT, 1]

        # Define transfer function
        tf_SOPDT = ctl.tf(num_SOPDT, den_SOPDT)

        # Simulation of the SOPDT model response to the input vector
        self.t_out_model_simulation, self.youtSOPDT_simulation = ctl.forced_response(tf_SOPDT, self.t_simulation,  self.input_simulation)
        
        
    def toDict(self):
            '''
            Function: toDict
            This function creates a JSON dictionary to store
            the parameters calculated and return the models
            in a single a defined format.
            '''
            json = {
                'T'     : self.time_constant,
                'K'     : self.dc_gain,
                'L'     : self.dead_time,
                'a'     : self.a_constant_time
            }
            print(json)
            return json
    
    def tune_controllers(self): 
        '''
        Function: tune_controllers
        This function executes four nested for loops to
        go trough each possibility of controller for each
        control mode, controller type, MS value, and degree of freedom.
        This function calls the main function of USORT tunning on each iteration
        and calculates and returns the possible tunned models.
        '''
        rule = usort()

        controllers = list()

        for control_mode in ["Regulatory", "Servo"]:
            for controller_type in ["PI", "PID"]:
                for ms_key in ["MS14", "MS16", "MS18", "MS2"]:
                    for DoF in [1, 2]:
                        try:
                            controller = rule.get_value(control_mode, 
                                           controller_type, 
                                           constant_a = self.a_constant_time, 
                                           ms_key = ms_key, 
                                           time_constant = self.time_constant, 
                                           dc_gain = self.dc_gain, 
                                           dead_time = self.dead_time, 
                                           DoF =  DoF)
                            controllers.append(controller)
                        except Exception as e:
                            print(e)
        return controllers
    

class FOPDT(Alfaro123c):
    '''
    Object: FOPDT
    Class used to process the respective parameters for first 
    order plus dead time model.
    '''
    def __init__(self, 
                time_vector : list = [],
                step_vector : list = [],
                resp_vector : list = [],
                ):
        '''
        Function: __init__
        This function initialize the variables and vectors
        to receive from the main class the processed values 
        and retrieve the respective model. 
        '''
        # FOPDT constants
        self.FOPDT_time_constant : float = 0
        self.FOPDT_dead_time : float = 0
        self.youtFOPDT_simulation : list = [] # FOPDT model simulated
        super().__init__(time_vector, step_vector, resp_vector)

        self.a_constant_time = 0
        self.dead_time = self.FOPDT_dead_time
        self.time_constant = self.FOPDT_time_constant
        

    def simulation(self):
        '''
        Function: simulation
        This function take the vectors with the model simulated
        and prints the respective behavior for FOPDT.
        '''
        plt.figure()
        plt.plot([x + self.dead_time for x in self.t_out_model_simulation], self.youtFOPDT_simulation+self.Yi, label='Model response')
        plt.plot(self.t_simulation, self.u_simulation, label='Input')
        plt.plot(self.t_simulation, self.y_sys_simulation, label='Plant output')
        plt.xlabel('Time')
        plt.ylabel('Output')
        plt.legend()
        plt.title('Model response')
        plt.grid(True)
        plt.show()

    def print(self):
        '''
        Function: print
        This mostly a debugging and informative function to
        print the respective parameter for the model is being executed.
        '''
        print("25% of the response time:", self.response_at_25percent_time.item())
        print("50% of the response time:", self.response_at_50percent_time.item())
        print("75% of the response time:", self.response_at_75percent_time.item())
        print("DC Gain: ",self.dc_gain)
        print("\n")

        print("*****FOPDT Parameters*****")
        print("FOPDT time constant:", self.time_constant)
        print("FOPDT dead time:", self.dead_time)
        print("FOPDT a time constant:", self.a_constant_time)
        print("\n")


    def tf(self):
        '''
        Function: tf
        This function uses the gain and time constant to create the transfer 
        function with the control library and return it as function's output.
        '''
        num = [self.dc_gain]  # Numerator
        den = [self.FOPDT_time_constant, 1]  # Denominator

        # Define transfer function
        tf_FOPDT = ctl.tf(num, den)
        return tf_FOPDT
            

    def toResponse(self):
        '''
        Function: toResponse 
        This function stores all the simulation vectors and creates
        a dictionary to return it as a response output.
        '''
        result = {
            'time'    :   self.t_out_model_simulation,       # Time vector
            'step'    :   self.u_simulation,       # Step vector
            'respo'   :   self.y_sys_simulation,       # Open-loop system response
            'm_respo' :   self.youtFOPDT_simulation  # Open-loop model-system response
        }
        return result
            
class SOPDT(Alfaro123c):
    '''
    Object: SOPDT
    Class used to process the respective parameters for second 
    order plus dead time model.
    '''
    def __init__(self, 
                time_vector : list = [],
                step_vector : list = [],
                resp_vector : list = [],
                ):
        '''
        Function: __init__
        This function initialize the variables and vectors
        to receive from the main class the processed values 
        and retrieve the respective model. 
        '''
        # FOPDT constants
        self.SOPDT_time_constant : float = 0
        self.SOPDT_dead_time : float = 0
        self.youtSOPDT_simulation : list = [] # SOPDT model simulated
        super().__init__(time_vector, step_vector, resp_vector)

        self.a_constant_time = 1
        self.dead_time = self.SOPDT_dead_time
        self.time_constant = self.SOPDT_time_constant

    def simulation_SOPDT(self):
        '''
        Function: simulation_SOPDT
        This function take the vectors with the model simulated
        and prints the respective behavior for SOPDT.
        '''
        plt.figure()
        plt.plot([x + self.dead_time for x in self.t_out_model_simulation], self.youtSOPDT_simulation+self.Yi, label='Model response')
        plt.plot(self.t_simulation, self.u_simulation, label='Input')
        plt.plot(self.t_simulation, self.y_sys_simulation, label='Plant output')
        plt.xlabel('Time')
        plt.ylabel('Output')
        plt.legend()
        plt.title('SOPDT Model response')
        plt.grid(True)
        plt.show()

    def print_SOPDT(self):
        '''
        Function: print_SOPDT
        This mostly a debugging and informative function to
        print the respective parameter for the model is being executed.
        '''
        print("*****SOPDT Parameters*****")
        print("SOPDT time constant:", self.time_constant)
        print("SOPDT dead time:", self.dead_time)
        print("FOPDT a time constant:", self.a_constant_time)
        print("\n")

    def tf_SOPDT(self):
        '''
        Function: tf_SOPDT
        This function uses the gain and time constant to create the transfer 
        function with the control library and return it as function's output.
        '''
        a_SOPDT = self.SOPDT_time_constant
        num_SOPDT = [self.dc_gain]
        den_SOPDT = [a_SOPDT*a_SOPDT, 2*a_SOPDT, 1]
        tf_SOPDT = ctl.tf(num_SOPDT, den_SOPDT)

        return tf_SOPDT
            
    def toResponse_SOPDT(self):
        '''
        Function: toResponse_SOPDT 
        This function stores all the simulation vectors and creates
        a dictionary to return it as a response output.
        '''
        result = {
            'time'    :   self.t_out_model_simulation,       # Time vector
            'step'    :   self.u_simulation,       # Step vector
            'respo'   :   self.y_sys_simulation,       # Open-loop system response
            'm_respo' :   self.youtSOPDT_simulation  # Open-loop model-system response
        }
        return result
            
class overdamped(Alfaro123c):
    '''
    Object: overdamped
    Class used to process the respective parameters for overdamped 
    model.
    '''
    def __init__(self, 
                time_vector : list = [],
                step_vector : list = [],
                resp_vector : list = [],
                ):
        '''
        Function: __init__
        This function initialize the variables and vectors
        to receive from the main class the processed values 
        and retrieve the respective model. 
        '''

        # FOPDT constants
        self.overdamped_time_constant : float = 0
        self.overdamped_dead_time : float = 0
        super().__init__(time_vector, step_vector, resp_vector)

        self.a_constant_time = (-0.6240*self.response_at_25percent_time + 0.9866*self.response_at_50percent_time -0.3626*self.response_at_75percent_time)/(0.3533*self.response_at_25percent_time - 0.7036*self.response_at_50percent_time + 0.3503*self.response_at_75percent_time)
        # Calculate Overdamped parameters 
        self.overdamped_time_constant = (self.response_at_75percent_time-self.response_at_25percent_time)/(0.9866 + 0.7036*self.a_constant_time)
        self.L_overdamped = self.response_at_75percent_time - (1.3421 + 1.3455*self.a_constant_time)*self.overdamped_time_constant
        
        if(self.L_overdamped.item() < 0):   #If FOPDT_dead_time is < 0, FOPDT_dead_time = 0
            self.L_overdamped = 0
        else:
            self.L_overdamped = self.L_overdamped.item()

        print(self.L_overdamped)
        print(self.a_constant_time)
        
        self.overdamped_time_constant = self.overdamped_time_constant.item()
        if self.a_constant_time > 1 or self.a_constant_time < 0:
            raise ValueError(f"Model for this value of 'a' time constant not found. Valid values are: 0 < a < 1")

        #self.a_constant_time = self.a_constant_time
        self.dead_time = self.overdamped_dead_time
        self.time_constant = self.overdamped_time_constant
        
        #Overdamped parameters and vectors
        Tao3 = self.overdamped_time_constant
        num_overdamped = [self.dc_gain]
        den_overdamped = [Tao3*Tao3*self.a_constant_time, Tao3*self.a_constant_time + Tao3, 1]
        tf_overdamped = ctl.tf(num_overdamped, den_overdamped)
        
        # Simulation od the overdamped model response to the input vector
        self.t_out_model_simulation, self.youtOverdamped_simulation = ctl.forced_response(tf_overdamped, self.t_simulation,  self.input_simulation)

    def simulation_overdamped(self):
        '''
        Function: simulation_overdamped
        This function take the vectors with the model simulated
        and prints the respective behavior for overdamped.
        '''
        plt.figure()
        plt.plot([x + self.dead_time for x in self.t_out_model_simulation], self.youtOverdamped_simulation+self.Yi, label='Model response')
        plt.plot(self.t_simulation, self.u_simulation, label='Input')
        plt.plot(self.t_simulation, self.y_sys_simulation, label='Plant output')
        plt.xlabel('Time')
        plt.ylabel('Output')
        plt.legend()
        plt.title('Overdamped Model response')
        plt.grid(True)
        plt.show()

    def print_overdamped(self):
        '''
        Function: print_overdamped
        This mostly a debugging and informative function to
        print the respective parameter for the model is being executed.
        '''
        print("25% of the response time:", self.response_at_25percent_time.item())
        print("50% of the response time:", self.response_at_50percent_time.item())
        print("75% of the response time:", self.response_at_75percent_time.item())
        print("DC Gain: ",self.dc_gain)
        print("\n")
        print("*****Overdamped Parameters*****")
        print("Overdamped 'a' time constant:", self.a_constant_time)
        print("Overdamped time constant:", self.time_constant)
        print("Overdamped dead time:", self.dead_time)
        print("\n")

    def tf_overdamped(self):
        '''
        Function: tf_overdamped
        This function uses the gain and time constant to create the transfer 
        function with the control library and return it as function's output.
        '''
        Tao3 = self.time_constant
        num_overdamped = [self.dc_gain]
        den_overdamped = [Tao3*Tao3*self.a_constant_time, Tao3*self.a_constant_time + Tao3, 1]
        tf_overdamped = ctl.tf(num_overdamped, den_overdamped)

        return tf_overdamped
            
    def toResponse_overdamped(self):
        '''
        Function: toResponse_overdamped
        This function stores all the simulation vectors and creates
        a dictionary to return it as a response output.
        '''
        result = {
            'time'    :   self.t_out_model_simulation,       # Time vector
            'step'    :   self.u_simulation,       # Step vector
            'respo'   :   self.y_sys_simulation,       # Open-loop system response
            'm_respo' :   self.youtOverdamped_simulation  # Open-loop model-system response
        }
        return result
    



