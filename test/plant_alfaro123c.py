import numpy as np
import pandas as pd
from io import StringIO


class Alfaro123c(): 
    def __init__(self,
                 ############### Raw data to identify the plant model
                 #time_vector: list=[], # Time vector to identify the plant model
                 #step_vector: list=[], # Step vector to identify the plant model
                 #resp_vector: list=[],  # Open-loop system response to identify the plant model

                 ############### Fractional order model in case it was calculated
                 alpha: float = 0,                 # Fractional order   (alpha)

                 ############### Common models constants
                 time_constant: float = 0,         # Main time constant (T)
                 dc_gain: float = 0, # Gain               (K)
                 dead_time: float = 0,    # Dead time          (L)
                 tp25 : float = 0,
                 tp50 : float = 0,
                 tp75 : float = 0,
                 K : float = 0,
                 a_t : float = 0,
                 Tao : float = 0,
                 L : float = 0,
                 TaoPD : float = 0,
                 LPD : float = 0,
                 tao_sobreamortiguado : float = 0,
                 L_sobreamortiguado : float = 0
                 ):

        #POMTM values
        aPO = 0.9102
        bPO = 1.2620

        #PDMTM values
        aPD = 0.5776
        bPD = 1.5552

        #Create dataframe with the vectors
        df = pd.read_excel('dataIDFOM2.xlsx')

        #Calculates Input derivative
        derivada_input = np.gradient(df['Entrada'], df['Tiempo'])
        df['Input_Derivative'] = derivada_input

        #Calculates Output derivative
        derivada_output = np.gradient(df['Salida'], df['Tiempo'])
        df['Output_Derivative'] = derivada_output.round()


        #Calculate output change
        #Looks for the derivative value 0 when the vector is stable
        derivative_final = df.loc[df['Output_Derivative'] == 0]
        Yf = derivative_final['Salida'].max()
        Yi = derivative_final['Salida'].min()
        deltaY = Yf - Yi
        

        #Calculate input change
        #Looks for the derivative value 0 when the vector is stable
        derivative_input = df.loc[df['Input_Derivative'] == 0]
        Uf = derivative_input['Entrada'].max()
        Ui = derivative_final['Entrada'].min()
        deltaU = Uf - Ui

        #Calculate K
        self.K = deltaY/deltaU

        #Uses the derivatives's max value to find the time point when input changes
        input_change = df.loc[[df['Input_Derivative'].idxmax()]]
        input_change_time = input_change['Tiempo'].max()
        

        #POMTM
        #Retrieves t25
        porcent25 = deltaY * 0.25                        #Gets the 25% of the response magnitude
        p25 = Yi + porcent25                             #Add the 25% to the first value, to have the 25% in terms of the response
        puntos25 = df[df['Salida'] < p25]                #Select all the values before the 25%
        self.tp25 = puntos25['Tiempo'].tail(1)           #Retrieves the last value before 25%
        diffs = df[df['Entrada'] != df['Entrada'].min()].min() #Finds the time point when the input changes
        tiempo_inicial = diffs['Tiempo']                 #Retrieves the time value when the input changes
        self.tp25 = self.tp25 - input_change_time        #Substract the initial time to the time found in order to have the time to reaches the 25% of the response
        
    
        #Retrieves t75
        porcent75 = deltaY * 0.75                       #Gets the 75% of the response magnitude
        p75 = df['Salida'].min() + porcent75            #Add the 75% to the first value, to have the 75% in terms of the response
        puntos75 = df[df['Salida'] < p75]               #Select all the values before the 75%
        self.tp75 = puntos75['Tiempo'].max()            #Retrieves the last value before 75%
        self.tp75 = self.tp75 - input_change_time       #Substract the initial time to the time found in order to have the time to reaches the 75% of the response
        

        #Retrieves t50
        porcent50 = deltaY * 0.50                       #Gets the 50% of the response magnitude
        p50 = df['Salida'].min() + porcent50            #Add the 50% to the first value, to have the 50% in terms of the response
        puntos50 = df[df['Salida'] < p50]               #Select all the values before the 50%
        self.tp50 = puntos50['Tiempo'].max()            #Retrieves the last value before 50%
        self.tp50 = self.tp50 - input_change_time       #Substract the initial time to the time found in order to have the time to reaches the 50% of the response
        
        

        #Calculate Tao and L POMTM
        self.Tao = aPO*(self.tp75 - self.tp25)
        self.L = bPO*self.tp25 + (1-bPO)*self.tp75
        if(self.L.item() < 0):                          #If L is < 0, L = 0
            self.L = 0
        else:
            self.L = self.L.item()
       

        #PDMTM
        self.TaoPD = aPD*(self.tp75 - self.tp25)
        self.LPD = bPD*self.tp25 + (1-bPD)*self.tp75
        if(self.LPD.item() < 0):                        #If L is < 0, L = 0
            self.LPD = 0
        else:
            self.LPD = self.LPD.item()

        #Sobreamortiguado
        self.a_t = (-0.6240*self.tp25 + 0.9866*self.tp50 -0.3626*self.tp75)/(0.3533*self.tp25 - 0.7036*self.tp50 + 0.3503*self.tp75)
        self.tao_sobreamortiguado = (self.tp75-self.tp25)/(0.9866 + 0.7036*self.a_t)
        self.L_sobreamortiguado = self.tp75 - (1.3421 + 1.3455*self.a_t)
        if(self.L_sobreamortiguado.item() < 0):         #If L is < 0, L = 0
            self.L_sobreamortiguado = 0
        else:
            self.L_sobreamortiguado = self.L_sobreamortiguado.item()


    def print(self):
        print("El tiempo al 25 es: ", self.tp25.item())
        print("El tiempo al 50 es: ", self.tp50.item())
        print("El tiempo al 75 es: ", self.tp75.item())
        print("K es: ",self.K)


        print("*****Parametros POMTM*****")
        print("El valor de Tao es: ", self.Tao.item())
        print("El valor del tiempo muerto es: ", self.L)


        print("*****Parametros PDMTM*****")
        print("El valor de Tao es: ", self.TaoPD.item())
        print("El valor del tiempo muerto es: ", self.LPD)
       

        print("*****Parametros Sbreamortiguado*****")
        print("El valor de la constante a es: ", self.a_t.item())
        print("El valor de Tao es: ", self.tao_sobreamortiguado.item())
        print("El valor del tiempo muerto es: ", self.L_sobreamortiguado)
 

test = Alfaro123c()
test.print()
