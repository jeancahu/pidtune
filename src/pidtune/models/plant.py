from ..rules import frac_order as _frac_order # Only rule it has by now
from ..utils.cronePadula2 import cronePadula2

from typeguard import typechecked
from subprocess import Popen, PIPE, STDOUT
from os import path, remove, rmdir, environ
import json

from control import tf, step_response # Transfer function, step response
from control.matlab import zpk2tf as zpk
import numpy as np

class FractionalOrderModel(): # TODO herence from generic plant model
    @typechecked
    def __init__(self,
                 ############### Raw data to identify the plant model
                 time_vector: list=[], # Time vector to identify the plant model
                 step_vector: list=[], # Step vector to identify the plant model
                 resp_vector: list=[],  # Open-loop system response to identify the plant model

                 ############### Fractional order model in case it was calculated
                 alpha: float = 0,                 # Fractional order   (alpha)

                 ############### Common models constants
                 time_constant: float = 0,         # Main time constant (T)
                 proportional_constant: float = 0, # Gain               (K)
                 dead_time_constant: float = 0,    # Dead time          (L)
                 ):

        ## Identify the plant model
        if (alpha or time_constant or proportional_constant or dead_time_constant):
            try:
                self.alpha = alpha
                self.T     = time_constant
                self.K     = proportional_constant
                self.L     = dead_time_constant
                self.IAE   = 0.0
                self.controllers = self.tune_controllers()
            except Exception:
                raise ValueError("Plant model wrong input values")

        else:
            if not (len(time_vector) and len(step_vector) and len(resp_vector)):
                raise ValueError("Plant model wrong input values, no vectors or constants")

            try:
                octalib_path = path.join(path.dirname(__file__), '../octalib/')
                octave_run = Popen(
                    ['octave-cli'],
                    cwd=octalib_path,
                    stdout=PIPE,
                    stdin=PIPE,
                    stderr=PIPE,
                    start_new_session=True,
                    env=environ.copy()
                )


                script = open(path.join(octalib_path, 'IDFOM.m'), 'r').readlines()
                script = """
% Run on execution start
version_info=ver("MATLAB");
try
  if (version_info.Name=="MATLAB")
    % Running on Matlab
  end
catch ME
  %% Running on Octave
  %% Octave load packages
  pkg load control
  pkg load symbolic
  pkg load optim
end

% Running initial module

clear;

%% Define
s=tf('s');

%% Global variables definition
global To vo Lo Ko ynorm unorm tnorm long tin tmax tu

%% Load data from thread cache
in_v1={};                                % time vector
in_v2={};                                % control signal vector
in_v3={};                                % controled variable vector
                """.format(
                    str(time_vector).replace(',',';'),
                    str(step_vector).replace(',',';'),
                    str(resp_vector).replace(',',';'),
                ) + "".join(script)

                octave_run.stdin.write(script.encode())
                octave_run.stdin.close()

                ### The two next lines will wait till the program ends. !IMPORTANT
                lines_std = [ line.decode() for line in octave_run.stdout.readlines()]
                lines = lines_std + [ line.decode() for line in octave_run.stderr.readlines()]

                error_code = octave_run.terminate()
                if error_code:
                    print("\n".join(lines))
                    raise Exception("Internal Octave/Matlab execution error: {}".format(error_code))

                results_list =[ json.loads(i) for i in lines_std[-3:] if "fractional_model" in i]
                if not len(results_list):
                    raise ValueError("Bad result for IDFOM excecution, verify your data.")

                results_dict = results_list[0]



                signals_matrix = [ i.replace("\n", "").replace("result_signals\t", '').split('\t') for i in lines if "result_signals" in i ]
                self.time_vector =       [ i[0] for i in signals_matrix ]  # Time vector
                self.step_vector =       [ i[1] for i in signals_matrix ]  # Step vector
                self.resp_vector =       [ i[2] for i in signals_matrix ]  # Open-loop system response
                self.model_resp_vector = [ i[3] for i in signals_matrix ]  # Open-loop model-system response

                ## Computed params
                self.alpha = results_dict["v"]
                self.T = results_dict["T"]
                self.K = results_dict["K"]
                self.L = results_dict["L"]
                self.IAE = results_dict["IAE"]

            except Exception as e:
                raise ValueError("Plant response wrong input vectors, verify your data, {}".format(str(e)))

        ## Tune controllers
        self.controllers = self.tune_controllers()

    def tf (self):
        """ Returns a control.tf transfer function (without dead time)
        """

        # Using CRONE Oustaloup to define Pm
        if (self.alpha <1):
            zz, pp, kk = cronePadula2(
                k = 1,
                v = -self.alpha,
            )

            mod = 1/zpk(zz,pp,kk)
        else:
            zz, pp, kk = cronePadula2(
                k = 1,
                v = self.alpha,
            )
            mod = zpk(zz,pp,kk)

        ## Convert mod into tf
        mod = tf(mod[0], mod[1])

        # l_tf = K*exp(-L*s)/(T*mod+1);
        return self.K/(self.T*mod+1) # No deadtime

    def tune_controllers(self):
        controllers = []

        if self.alpha > 1.6:
            valid_ctls = ('PID',)
        else:
            valid_ctls = _frac_order.valid_controllers

        for ctype in valid_ctls:
            for Ms in _frac_order.valid_Ms:
                temp = _frac_order.tuning(
                    self.alpha,
                    self.T,
                    self.K,
                    self.L,
                    Ms,
                    ctype
                )
                if temp:
                    controllers.append(temp)
        return controllers

    def toDict(self):
        return {
            'alpha' : self.alpha,
            'T'     : self.T,
            'K'     : self.K,
            'L'     : self.L,
            'IAE'   : self.IAE
        }

    def toResponse(self):
        try:
            result = {
                'time'    :   self.time_vector,       # Time vector
                'step'    :   self.step_vector,       # Step vector
                'respo'   :   self.resp_vector,       # Open-loop system response
                'm_respo' :   self.model_resp_vector  # Open-loop model-system response
            }
        except Exception as e:

            t, y = step_response(self.tf())

            result = {
                'time'    :   list(np.add(self.L, t)),       # Time vector
                'step'    :   [],                     # Step vector
                'respo'   :   [],                     # Open-loop system response
                'm_respo' :   list(y)  # Open-loop model-system response
        }

        return result

    def __str__(self):
        return str(self.toDict())
