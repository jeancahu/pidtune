from typeguard import typechecked
from control import tf
from control.matlab import zpk2tf as zpk, trapz, pade, step
from .cronePalula2 import cronePalula2 as APC
#import numpy as np

@typechecked
def IDFOM(
        T: float,        # Plant main time constant
        v: float,        # Plant fractional order
        L: float,        # Plant dead time
        tnorm: list,     # Normalized time vector
        ynorm: list,     # Normalized plantresponse vector
        tin: float,      # Time the step input is applied to the plant
        flagtin: int,    # Indicates the tin position in data vector
        opp: bool = True # Flag TODO
) -> any: # TODO

    s = tf('s') # Defines the S operator
    if v < 1:
        z,p,k = APC(1,-v,0.001,1000)
        temp_l = zpk(z,p,k);
        Gmm=1/tf(temp_l[0],temp_l[1])
    else:
        z,p,k = APC(1,v,0.001,1000)
        temp_l = zpk(z,p,k);
        Gmm=tf(temp_l[0],temp_l[1])

    Gm0 = 1*pade(L+tin, 18)/(v*Gmm+1)
    yout, i = step(Gm0,tnorm);

    if opp:
        tnorm_temp = tnorm[:flagtin]
        ynorm_temp = ynorm[:flagtin]
        yout_temp = yout[:flagtin]

        return trapz(tnorm, [abs(i-j) for i,j in zip(yout-ynorm)]) - \
            trapz(tnorm_temp, [abs(i-j) for i,j in zip(yout_temp-ynorm_temp)])

    return trapz(tnorm, [abs(i-j) for i,j in zip(yout-ynorm)])
