from typeguard import typechecked
from . import controller as cnt, plant as pln

class CloseLoop ():
    @typechecked
    def __init__(
            self,
            controller: cnt.Controller,
            plant: pln.FractionalOrderModel ## TODO generic plant
    ):

        print(controller)
        print(plant)

    def step_response(self):
        pass

class OpenLoop ():
    pass
