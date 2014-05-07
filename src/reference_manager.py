"""
For now this file will just provide a single function that takes in a dsys
and a time vector, and returns a full state and input reference trajectory.
Later on I can update this file to do things like subscribe to some topic, store
trajectory data, interpolate data, offset times from ROS time to optimization
time, etc.
"""
import roslib; roslib.load_manifest('receding_planar_sys')
from std_msgs.msg import Time
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from scipy.interpolate import interp1d
import rospy
import numpy as np
from math import pi
import threading
# local imports:
import system_definition as sd

MAX_POINTS = 300

class RefManager( object ):
    """
    instantiator takse following args:
    rx = half-width of superellipse
    ry = half-height of superellipse
    n = power of superellipse
    tper = time of traversal of superellipse
    tau = inverse of the exponent of lead-in (0.0 or None => no lead in, higher
                number => slower lead in)
    interactive = if true, use an interactive reference generator
    """
    def __init__(self, rx=1.0/2.0, ry=0.25/2.0, n=2.5, tper=10.0, r0=1.0,
                 tau=None, interactive=False):
        self.rx = rx
        self.ry = ry
        self.r0 = r0
        self.n = n
        self.tper = tper
        if tau is not None:
            self.tau = tau
        else:
            self.tau=None
        # time param (only used in interactive case)
        self.tstart = -1.0
        self.interactive = interactive
        if interactive:
            # setup interactive reference traj generator
            self.start_time_sub = rospy.Subscriber("start_time", Time, self.timecb)
            self.mass_ref_sub = rospy.Subscriber("mass_ref_point", PointStamped, self.pointcb)
            self.mutex = threading.Lock()
            self.X0 = np.hstack(([0,sd.h0-self.r0,0,self.r0], np.zeros(4,)))
            self._tvec = np.array([-0.1, 0.0])
            self._xvec = np.array([self.X0, self.X0])
            self.update_interp()
            self.calc_reference_traj = self.interactive_reference_traj
        else:
            self.calc_reference_traj = self.parametric_reference_traj
        return

    def interactive_reference_traj(self, dsys, tvec):
        Xref = np.zeros((len(tvec), dsys.nX))
        # Uref = np.zeros((len(tvec)-1, dsys.nU))
        # iterate through all times in tvec, calculate the interpolated X value:
        for i,t in enumerate(tvec):
            try:
                Xref[i] = self.interp_ref_state(t)
            except ValueError:
                # must have requested a state outside of valid range
                return None
        Uref = Xref[1:, dsys.system.nQd:dsys.system.nQ]
        return Xref,Uref

    def interp_ref_state(self, t):
        with self.mutex:
            return self._fint(t)
        
    def update_interp(self):
        self._fint = interp1d(self._tvec, self._xvec, copy=True, kind='linear', axis=0)
        
    def reset_interps(self):
        self.tstart = -1
        if self.interactive:
            self._tvec = np.array([-0.1, 0.0])
            self._xvec = np.array([self.X0, self.X0])

    def timecb(self, t):
        self.tstart = t.data.to_sec()

    def pointcb(self, point):
        # convert the time and add to time vector
        self._tvec = np.append(self._tvec, point.header.stamp.to_sec() - self.tstart)
        # get dynamic ref configuration in the plane:
        qd = np.array([point.point.x, point.point.y])
        # now fill in inverse-kinematic configuration variables:
        qk = np.array([qd[0], sd.h0 - qd[1]])
        Q = np.hstack((qd,qk))
        # now fill in velocity and momentum:
        X = np.hstack((Q, np.zeros(len(Q),)))
        self._xvec = np.append(self._xvec, [X], axis=0)
        # now trim vectors if necessary:
        if len(self._tvec) > MAX_POINTS:
            self._tvec = np.delete(self._tvec, 0)
            self._xvec = np.delete(self._xvec, 0, axis=0)
        # now with a lock, let's update the interp:
        with self.mutex:
            self.update_interp()
        return

    def parametric_reference_traj(self, dsys, tvec):
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
            qd[i, ymi] = np.abs(np.sin(th))**(2.0/self.n)*self.ry*np.sign(np.sin(th))
            # account for offset:
            qd[i, ymi] += (sd.h0-self.r0) - self.ry
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


