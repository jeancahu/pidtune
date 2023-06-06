import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import control as ctl
import scipy.signal as signal
from io import StringIO
from scipy.signal import savgol_filter
from os import path


class Alfaro123c(): 
    def __init__(self, 
                 response_at_25percent_time : float = 0,
                 response_at_50percent_time : float = 0,
                 response_at_75percent_time : float = 0,
                 dc_gain : float = 0,
                 a_constant_time : float = 0,
                 FOPDT_time_constant : float = 0,
                 FOPDT_dead_time : float = 0,
                 SOPDT_time_constant : float = 0,
                 SOPDT_dead_time : float = 0,
                 overdamped_time_constant : float = 0,
                 overdamped_dead_time : float = 0,
                 time_vector : list = [],
                 step_vector : list = [],
                 resp_vector : list = [],
                 ):

        df = pd.DataFrame({'Tiempo': time_vector, 'Entrada': step_vector, 'Salida': resp_vector})

        #FOPDT values
        a_FOPDT = 0.9102
        b_FOPDT = 1.2620

        #SOPDT values
        a_SOPDT = 0.5776
        b_SOPDT = 1.5552

        #Creates dataframe with the dynamic vectors
        #df = pd.read_excel('GUNT.xlsx')

        #Calculates Input derivative
        filter_input = savgol_filter(df['Entrada'], window_length=51, polyorder=3, deriv=0)
        derivada_input = np.abs(np.gradient(filter_input, df['Tiempo']))
        df['Input_Derivative'] = derivada_input

        #Calculates Output derivative
        filter_output =  savgol_filter(df['Salida'], window_length=51, polyorder=3, deriv=0)
        derivada_output = np.abs(np.gradient(filter_output, df['Tiempo']))
        df['Output_Derivative'] = derivada_output

        #Calculates the 5% of the response change using the filtered output
        delta_output = np.subtract(np.max(filter_output, axis=0), np.min(filter_output, axis=0))
        outputChange_5percent = np.multiply(0.05, delta_output) # Gets the 'Delta Y' 10%

        #Gets the max_value_output_index of the max value
        max_value_output_index =  np.where(derivada_output == np.max(derivada_output, axis=0))[0][0]
        #print(max_value_output_index)

        #Save the max and min value of the filtered response
        maxim_output = np.max(filter_output, axis=0)
        min_output = np.min(filter_output, axis=0)
        
        #Separates the output vector, before and after the maxim value
        before_transient_response = df.loc[:max_value_output_index]
        after_transient_response = df.loc[max_value_output_index:]

        #Define the vector with only the output values
        before_transient_response = before_transient_response['Salida']
        after_transient_response = after_transient_response['Salida']

        ### Verify if the signal is Increasing or decreasing
        if np.mean(before_transient_response) < np.mean(after_transient_response):
            ## If the mean of the first values is less than the mean of the last values
            ## then the signal is Increasing

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
        Yf = np.mean(valores_finales_estables)
        Yi = np.mean(valores_iniciales_estables)

        #Calculates change of the system response 
        SystemResponse_change = Yf - Yi
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
            p25 = Yi + porcent25                        #Add the 25% to the first value, to have the 25% in terms of the response
            puntos25 = df[df['Salida'] < p25]           #Select all the values before the 25%
            self.response_at_25percent_time = puntos25['Tiempo'].max()  #Retrieves the last value before 25%
            self.response_at_25percent_time = self.response_at_25percent_time - input_change_time  #Substract the initial time to the time found in order to have the time to reaches the 25% of the response
            
            #Retrieves t75
            porcent75 = SystemResponse_change * 0.75     #Gets the 75% of the response magnitude
            p75 = df['Salida'].min() + porcent75         #Add the 75% to the first value, to have the 75% in terms of the response
            puntos75 = df[df['Salida'] < p75]            #Select all the values before the 75%
            self.response_at_75percent_time = puntos75['Tiempo'].max()  #Retrieves the last value before 75%
            self.response_at_75percent_time = self.response_at_75percent_time - input_change_time #Substract the initial time to the time found in order to have the time to reaches the 75% of the response
            
            #Retrieves t50
            porcent50 = SystemResponse_change * 0.50        #Gets the 50% of the response magnitude
            p50 = df['Salida'].min() + porcent50            #Add the 50% to the first value, to have the 50% in terms of the response
            puntos50 = df[df['Salida'] < p50]               #Select all the values before the 50%
            self.response_at_50percent_time = puntos50['Tiempo'].max()  #Retrieves the last value before 50%
            self.response_at_50percent_time = self.response_at_50percent_time - input_change_time #Substract the initial time to the time found in order to have the time to reaches the 50% of the response
        
        else:
            # Decreasing case 
            #Retrieves t25
            porcent25 = SystemResponse_change_positivo * 0.25 #Gets the 25% of the response magnitude
            p25 = Yi - porcent25                             #Add the 25% to the first value, to have the 25% in terms of the response
            puntos25 = df[df['Salida'] > p25]                #Select all the values before the 25%
            self.response_at_25percent_time = puntos25['Tiempo'].max()#Retrieves the last value before 25%
            self.response_at_25percent_time = self.response_at_25percent_time - input_change_time #Substract the initial time to the time found in order to have the time to reaches the 25% of the response
            
            porcent75 = SystemResponse_change_positivo * 0.75 #Gets the 75% of the response magnitude
            p75 = Yi - porcent75            #Add the 75% to the first value, to have the 75% in terms of the response
            puntos75 = df[df['Salida'] > p75]  #Select all the values before the 75%
            self.response_at_75percent_time = puntos75['Tiempo'].max()  #Retrieves the last value before 75%
            self.response_at_75percent_time = self.response_at_75percent_time - input_change_time  #Substract the initial time to the time found in order to have the time to reaches the 75% of the response

            #Retrieves t50
            porcent50 = SystemResponse_change_positivo * 0.50  #Gets the 50% of the response magnitude
            p50 = Yi - porcent50            #Add the 50% to the first value, to have the 50% in terms of the response
            puntos50 = df[df['Salida'] > p50]        #Select all the values before the 50%
            self.response_at_50percent_time = puntos50['Tiempo'].max() #Retrieves the last value before 50%
            self.response_at_50percent_time = self.response_at_50percent_time - input_change_time  #Substract the initial time to the time found in order to have the time to reaches the 50% of the response


        #Calculate FOPDT_time_constant and FOPDT_dead_time 
        self.FOPDT_time_constant = a_FOPDT*(self.response_at_75percent_time - self.response_at_25percent_time)
        self.FOPDT_dead_time = b_FOPDT*self.response_at_25percent_time + (1-b_FOPDT)*self.response_at_75percent_time
        if(self.FOPDT_dead_time.item() < 0):      #If FOPDT_dead_time is < 0, FOPDT_dead_time = 0
            self.FOPDT_dead_time = 0
        else:
            self.FOPDT_dead_time = self.FOPDT_dead_time.item()
       
        self.FOPDT_time_constant = self.FOPDT_time_constant.item()

        #SOPDT
        self.SOPDT_time_constant = a_SOPDT*(self.response_at_75percent_time - self.response_at_25percent_time)
        self.SOPDT_dead_time = b_SOPDT*self.response_at_25percent_time + (1-b_SOPDT)*self.response_at_75percent_time
        if(self.SOPDT_dead_time.item() < 0):      #If FOPDT_dead_time is < 0, FOPDT_dead_time = 0
            self.SOPDT_dead_time = 0
        else:
            self.SOPDT_dead_time = self.SOPDT_dead_time.item()
        
        self.SOPDT_time_constant = self.SOPDT_time_constant.item()

        #overdamped
        self.a_constant_time = (-0.6240*self.response_at_25percent_time + 0.9866*self.response_at_50percent_time -0.3626*self.response_at_75percent_time)/(0.3533*self.response_at_25percent_time - 0.7036*self.response_at_50percent_time + 0.3503*self.response_at_75percent_time)
        self.overdamped_time_constant = (self.response_at_75percent_time-self.response_at_25percent_time)/(0.9866 + 0.7036*self.a_constant_time)
        self.L_overdamped = self.response_at_75percent_time - (1.3421 + 1.3455*self.a_constant_time)*self.overdamped_time_constant
        #print(self.L_overdamped)
        if(self.L_overdamped.item() < 0):   #If FOPDT_dead_time is < 0, FOPDT_dead_time = 0
            self.L_overdamped = 0
        else:
            self.L_overdamped = self.L_overdamped.item()
        
        self.overdamped_time_constant = self.overdamped_time_constant.item()

        self.a_constant_time = self.a_constant_time.item()

        if(self.a_constant_time < 0):
            self.a_constant_time = 0
        
        
        # Simulation
        # Time vector to simulate the system
        t = df['Tiempo'].values

        # Input vector to simulate the system
        u = df['Entrada'].values

        ######Input vector to the simulation######
        # Looks for the diference between consecutive positions
        # Adds an input simulation vector "Cambio" that contains tha magnitude of the input changes
        df['Cambio'] = df['Entrada'].diff().fillna(0)

        # Finds the first non zero value
        first_non_zero = df['Cambio'].ne(0).idxmax()

        # Replace the non useful values by NaN
        df.loc[first_non_zero:, 'Cambio'] = np.where(df.loc[first_non_zero:, 'Cambio'] == 0, np.nan, df.loc[first_non_zero:, 'Cambio'])

        # Fill NaN values with the last valid value
        df['Cambio'].fillna(method='ffill', inplace=True)

        input_simulation = df['Cambio'].values
        ######################################

        #FOPDT Simulation
        # Define numerator and denominator to the transfer function without deadtime
        num = [self.dc_gain]  # Numerator
        den = [self.FOPDT_time_constant, 1]  # Denominator

        # Define transfer function
        tf_FOPDT = ctl.tf(num, den)
        
        # Output vector
        y_sys = df['Salida'].values
        #print(y_sys)

        # Simulate the FOPDT model response to the input vector
        t_out, y = ctl.forced_response(tf_FOPDT, t, input_simulation)
        
        plt.figure()
        plt.plot([x + self.FOPDT_dead_time for x in t_out], y+Yi, label='Model response')
        plt.plot(t, u, label='Input')
        plt.plot(t, y_sys, label='Plant output')
        plt.xlabel('Time')
        plt.ylabel('Output')
        plt.legend()
        plt.title('Model response')
        plt.grid(True)
        plt.show()

        # SOPDT parameters and vectors
        a_SOPDT = self.SOPDT_time_constant
        num_SOPDT = [self.dc_gain]
        den_SOPDT = [a_SOPDT*a_SOPDT, 2*a_SOPDT, 1]
        tf_SOPDT = ctl.tf(num_SOPDT, den_SOPDT)

        # Simulation of the SOPDT model response to the input vector
        t_out2, y2 = ctl.forced_response(tf_SOPDT, t,  input_simulation)
        
        plt.figure()
        plt.plot([x + self.SOPDT_dead_time for x in t_out2], y2+Yi, label='Model response')
        plt.plot(t, u, label='Input')
        plt.plot(t, y_sys, label='Plant output')
        plt.xlabel('Time')
        plt.ylabel('Output')
        plt.legend()
        plt.title('Model response')
        plt.grid(True)
        plt.show()

        #Overdamped parameters and vectors
        Tao3 = self.overdamped_time_constant
        num_overdamped = [self.dc_gain]
        den_overdamped = [Tao3*Tao3*self.a_constant_time, Tao3*self.a_constant_time + Tao3, 1]
        tf_overdamped = ctl.tf(num_overdamped, den_overdamped)
        
        # Simulation od the overdamped model response to the input vector
        t_out3, y3 = ctl.forced_response(tf_overdamped, t,  input_simulation)

        plt.figure()
        plt.plot([x + self.L_overdamped for x in t_out3], y3+Yi, label='Model response')
        plt.plot(t, u, label='Input')
        plt.plot(t, y_sys, label='Plant output')
        plt.xlabel('Time')
        plt.ylabel('Output')
        plt.legend()
        plt.title('Model response')
        plt.grid(True)
        plt.show()

    def print(self):
        print("25% of the response time:", self.response_at_25percent_time.item())
        print("50% of the response time:", self.response_at_50percent_time.item())
        print("75% of the response time:", self.response_at_75percent_time.item())
        print("DC Gain: ",self.dc_gain)
        print("\n")

        print("*****FOPDT Parameters*****")
        print("FOPDT time constant:", self.FOPDT_time_constant)
        print("FOPDT dead time:", self.FOPDT_dead_time)
        print("\n")

        print("*****Parametros SOPDT*****")
        print("SOPDT time constant:", self.SOPDT_time_constant)
        print("SOPDT dead time:", self.SOPDT_dead_time)
        print("\n")

        print("*****Overdamped Parameters*****")
        print("Overdamped 'a' time constant:", self.a_constant_time)
        print("Overdamped time constant:", self.overdamped_time_constant)
        print("Overdamped dead time:", self.L_overdamped)
        print("\n")


