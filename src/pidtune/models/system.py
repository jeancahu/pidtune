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
        pade_delay_cf = pade(
            self.plant.toDict()['L'],
            4 # Order
        )
        pade_delay = tf(pade_delay_cf[0], pade_delay_cf[1])

        My_r = ctl_tf*pln_tf*pade_delay / ( 1 + ctl_tf*pln_tf*pade_delay )

        ts, xs = step(My_r)

        ## If the simulation time is less than L, simulate again for a longer period
        if (np.max(ts) < self.plant.L):
            ts, xs = step(My_r, np.arange(0, 30 * self.plant.L, self.plant.L/10))

        stationary = len(
            xs[ abs(xs - 1) < 0.001 ]
        )

        ## If there are not stationary points in the vector, let's simulate for the doble of time
        if stationary < (0.05)*len(ts):
            ts, xs = step(My_r, np.arange(0, 2*ts[-1], self.plant.L/10))


        t_pade_vect, y_pade_vect = ts, xs
        stationary = len(
            xs[ abs(xs - 1) < 0.001 ]
        )
        ts = ts[:(len(xs) - stationary)] # Drop stationary data
        xs = xs[:(len(xs) - stationary)]


        series = int(ts[-1]/self.plant.L)
        L_shift_factor = 50
        if series > 30:
            series = 30

        ts = ts[ts > series * self.plant.L] # Drop stationary data
        xs = xs[(len(xs)-len(ts)):]

        series_t = np.arange(0, series * self.plant.L, self.plant.L/L_shift_factor, dtype=float)
        series_y = np.full(len(series_t), 0.0, dtype=float)
        y_temp = np.full(len(series_t), 1.0, dtype=float)

        for i in range(series):
            y_temp, l_time, l_x0 = lsim(ctl_tf*pln_tf, y_temp, series_t)
            y_temp = [0] * L_shift_factor + list(y_temp[:len(y_temp) -L_shift_factor])

            if i%2:
                series_y = np.subtract(series_y, y_temp)
            else:
                series_y = np.add(series_y, y_temp)

        ## Return full time vector, full yout vector
        return (
            list(series_t) + list(ts), # time vector
            list(series_y) + list(xs), # Y vector
            list(t_pade_vect),
            list(y_pade_vect),
        )
