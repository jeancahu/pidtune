from typeguard import typechecked
from control import tf

class Controller():
    @typechecked
    def __init__(
            self,
            ctype: str,
            Ms: str,
            n_kp: float,
            n_ti: float,
            n_td: float,
            kp: float,
            ti: float,
            td: float,

            ## PID ODoF by default
            action: float = 1,
            filter_constant: float = 0.1,
            beta: float = 1, # TODO
            gamma: float = 0, # TODO
    ):
        self.ctype = ctype
        self.Ms = Ms
        self.n_kp = n_kp
        self.n_ti = n_ti
        self.n_td = n_td
        self.kp = kp
        self.ti = ti
        self.td = td

        self.action = action
        self.filter_constant = filter_constant

        # raise Exception("Wrong value") #TODO
        # raise ValueError("Wrong value") #TODO

    # Print json format __str__ etc

    def tf(self):
        s = tf('s')
        return self.action * self.kp*( 1 + (1/(self.ti*s)) + ((self.td*s)/(1+self.filter_constant*self.td*s)))

    def toDict(self):
        return {
            'ctype' : self.ctype,
            'Ms'    : self.Ms,
            'n_kp'  : self.n_kp,
            'n_ti'  : self.n_ti,
            'n_td'  : self.n_td,
            'kp'    : self.kp,
            'ti'    : self.ti,
            'td'    : self.td
        }

    def __str__(self):
        return str(self.toDict())
