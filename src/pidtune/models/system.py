from typeguard import typechecked
from . import controller as cnt, plant as pln
import numpy as np
from control import step_response as step, tf
from control.matlab import pade, lsim

class OpenLoop ():
    pass

class ClosedLoop ():
    @typechecked
    def __init__(
            self,
            controller: cnt.Controller,
            plant: pln.FractionalOrderModel ## TODO generic plant
    ):

        self.controller = controller
        self.plant = plant

    def step_response(
            self,
            magnitude: float = 1.0
    ):
        ctl_tf = self.controller.tf()
        pln_tf = self.plant.tf()

        ## Define pade delay order 4
        pade_delay_cf = pade(
            self.plant.toDict()['L'],
            4 # Order
        )
        pade_delay = tf(pade_delay_cf[0], pade_delay_cf[1])

        ## ClosedLoop using PADE deadtime approximation
        My_r = ctl_tf*pln_tf*pade_delay / ( 1 + ctl_tf*pln_tf*pade_delay )
        My_d = pln_tf*pade_delay / ( 1 + ctl_tf*pln_tf*pade_delay )

        ## Time vectorn and Y(s) vector
        ts, xs = step(My_r)

        ## If the simulation time is less than L, simulate again for a longer period
        if (np.max(ts) < self.plant.L):
            ts, xs = step(My_r, np.arange(0, 30 * self.plant.L, self.plant.L/10))

        # ## If there are not stationary points in the vector, let's simulate for two times the previuos time
        for j in range(4):
            i = np.abs(np.subtract(xs, 1))
            temporary = np.sum(np.array(i > 0.00005, dtype=int))
            if (temporary < 0.9*len(i)):
                break
            ts, xs = step(My_r, np.arange(0, 2*ts[-1], self.plant.L/10)) # FIXME

        ts = ts[:temporary] # Drop stationary data
        xs = xs[:temporary]

        ## Regulatory response
        ts_reg, xs_reg = step(My_d, ts)

        series = int(ts[-1]/self.plant.L)
        L_shift_factor = 50
        if series > 30:
            series = 30

        ts = ts[ts > series * self.plant.L] # Drop PADE transitory data
        xs = xs[(len(xs)-len(ts)):]
        xs_reg = xs_reg[(len(xs_reg)-len(ts)):]

        ## Servo
        series_t = np.arange(0, series * self.plant.L, self.plant.L/L_shift_factor, dtype=float)
        series_y = np.full(len(series_t), 0.0, dtype=float)
        y_temp = np.full(len(series_t), 1.0, dtype=float)


        ## Regulatory first serie
        y_temp_reg, l_time_reg, l_x0_reg = lsim(pln_tf, y_temp, series_t)
        y_temp_reg = [0] * L_shift_factor + list(y_temp_reg[:len(y_temp_reg) -L_shift_factor])
        series_y_reg = np.full(len(series_t), 0.0, dtype=float) ## Regulatory response
        series_y_reg = np.add(series_y_reg, y_temp_reg)

        for i in range(series):
            # Servo
            y_temp, l_time, l_x0 = lsim(ctl_tf*pln_tf, y_temp, series_t)
            y_temp = [0] * L_shift_factor + list(y_temp[:len(y_temp) -L_shift_factor])

            # Regulatory
            y_temp_reg, l_time_reg, l_x0_reg = lsim(ctl_tf*pln_tf, y_temp_reg, series_t)
            y_temp_reg = [0] * L_shift_factor + list(y_temp_reg[:len(y_temp_reg) -L_shift_factor])

            if i%2:
                series_y = np.subtract(series_y, y_temp)
                series_y_reg = np.add(series_y_reg, y_temp_reg)
            else:
                series_y = np.add(series_y, y_temp)
                series_y_reg = np.subtract(series_y_reg, y_temp_reg)

        ## Concatenate servo
        series_y = list(series_y) + list(xs)

        ## Reduce disturbance to 10%
        series_y_reg = list(series_y_reg) + list(xs_reg)
        series_y_reg = np.multiply(0.1, series_y_reg)

        ## concatenate time
        series_t = list(series_t) + list(ts)

        ## Calculate IAE:
        y_error =  np.abs(np.subtract(1, series_y))
        IAE = np.trapz(y_error, series_t)

        ## Calculate IAE_reg:
        y_error =  np.abs(series_y_reg)
        IAE_reg = np.trapz(y_error, series_t)

        ## Return full time vector, full yout vector
        return (
            list(series_t), # time vector
            list(series_y), # Y vector
            list(series_y_reg), # Y vector
            IAE,
            IAE_reg
        )
