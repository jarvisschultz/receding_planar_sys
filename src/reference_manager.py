"""
For now this file will just provide a single function that takes in a dsys
and a time vector, and returns a full state and input reference trajectory.
Later on I can update this file to do things like subscribe to some topic, store
trajectory data, interpolate data, offset times from ROS time to optimization
time, etc.
"""
import numpy as np
from math import pi
import system_definition as sd


class RefManager( object ):
    """
    instantiator takse following args:
    rx = half-width of superellipse
    ry = half-height of superellipse
    n = power of superellipse
    tper = time of traversal of superellipse
    tau = inverse of the exponent of lead-in (0.0 or None => no lead in, higher
                number => slower lead in)
    """
    def __init__(self, rx=1.0/2.0, ry=0.25/2.0, n=2.5, tper=10.0, tau=None):
        self.rx = rx
        self.ry = ry
        self.n = n
        self.tper = tper
        if tau is not None:
            self.tau = tau
        else:
            self.tau=None
        return

    def calc_reference_traj(self, dsys, tvec):
        # build empty array for the dynamic configurations as a function of time
        qd = np.zeros((len(tvec), dsys.system.nQd))
        xmi = dsys.system.get_config('xm').index
        ymi = dsys.system.get_config('ym').index
        # fill out desired dynamic vars:
        for i,t in enumerate(tvec):
            if t <= self.tper:
                if self.tau:
                    ttmp = t*(1-np.exp(-t*(1/self.tau)))
                else:
                    ttmp = t
                th = ttmp*2*pi/self.tper + pi/2
            else:
                th = 2*pi + pi/2
            qd[i, xmi] = np.abs(np.cos(th))**(2.0/self.n)*self.rx*np.sign(np.cos(th))
            qd[i, ymi] = np.abs(np.sin(th))**(2.0/self.n)*self.ry*np.sign(np.sin(th)) - self.ry #subtract ry to ensure initial length is 1 meter
        # now let's calculate the inverse-kinematics based version of the kinematic
        # variables
        qk = np.zeros((len(tvec), dsys.system.nQk))
        xri = dsys.system.get_config('xr').index - dsys.system.nQd
        ri = dsys.system.get_config('r').index - dsys.system.nQd
        for i,t in enumerate(tvec):
            qk[i, xri] = qd[i, xmi]
            qk[i, ri] = sd.h0 - qd[i, ymi]
        Q = np.hstack((qd,qk))
        # fill out Uref to be offset from kinematic inputs:
        Uref = qk[1:]
        if len(tvec) != len(dsys.time):
            X = np.zeros((len(tvec), dsys.nX))
            U = np.zeros((len(tvec)-1, dsys.nU))
            X[:,0:dsys.system.nQ] = Q
            U = Uref.copy()
        else:
            X,U = dsys.build_trajectory(Q=Q, rho=Uref)
        return X,U


