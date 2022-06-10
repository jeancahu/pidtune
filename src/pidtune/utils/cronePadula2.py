from typeguard import typechecked
from control import tf
from control.matlab import zpk2tf as zpk, dcgain, bode
import numpy as np

@typechecked
def cronePadula2 (
        k: float,          # Gain
        v: float,          # Fractional order
        wl: float = 0.001, # Floor frequency
        wh: float = 1000,  # Ceil frequency
        n : int = 8        # Approximation order (n)
) -> tuple:                # Output transfer function
    """ CronePadula2 approach a Laplace transform for a fractional order
    differentiator or integrator.
    It returns a list of zeros, poles and a gain for the equivalent
    transfer function.

    Algorithm source:
    Oustaloup, A., Levron, F., Mathieu, B. y Nanot, F.M. (2000) Frequency-
      Band Complex Noninteger Differentiator: Characterization and
      Synthesis. IEEE TRANSACTIONS ON CIRCUITS AND SYSTEMS,I: FUNDAMENTAL
      THEORY AND APPLICATIONS, VOL.47, NO.1, pp.25-39.

    :param gain: Float which represents the model gain
    :type gain: float

    :param alpha: Fractional order
    :type alpha: float

    :param wl:
    :type wl:

    :param wh:
    :type wh:

    :returns: A transfer function which approximates the real factorial one
    :rtype: tf
    """

    # Round to the floor value
    f = int(v)
    vf = v - f

    z = [] # Zeros
    p = [] # Poles

    alpha = np.power(wh/wl,vf/n)
    eta   = np.power(wh/wl,(1-vf)/n)

    z.append(wl * np.sqrt(eta))
    p.append(z[0]*alpha)

    for x in range(n-1): # n-1 due first case already done
        z.append(p[-1]*eta)
        p.append(z[-1]*alpha)


    negz = [ -x for x in z ]
    negp = [ -x for x in p ]

    num, den = zpk(negz, negp, 1)
    C = tf(num, den)

    k1 = k / dcgain(C)
    C= k1*C

    wm = np.sqrt(wl*wh)

    bode_result = bode(C, [wm])
    k2=(k*np.power(wm,vf))/bode_result[0][0] ## FIXME gain matrix
    k=k2*k1

    if (v>0):
        return (negz + [0]*f, negp, k)
    return (negz, negp + [0]*f, k)
