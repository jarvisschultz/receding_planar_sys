from numpy import (
    array, sqrt, pi, arctan, dot, sin, cos, exp, zeros, random, floor, ones,
    mean, cov, linspace, sum, isnan, where, hstack, linalg, eye, diag
    )
import copy
import hashlib
import tools
import scipy.io as sio
import os
import trep
from trep import tx, ty, tz, rx, ry, rz
import trep.discopt as discopt


class EKF( object ):
    """
    A class for general extended kalman filters
    """
    def __init__(self, name, X0, Dt, proc_cov, meas_cov, Hk=None,
                 num=1000, L=100):
        """
        INPUTS:
        name ~ string identifying filter
        X0 ~ initial state
        Dt ~ timestep
        proc_cov ~ process covariance
        meas_cov ~ measurement covariance 
        Hk ~ matrix for measurement model
        num ~ number of particles
        L ~ number of timesteps
        """
        self.name = name
        self.X0 = X0
        self.num = num
        self.dim = len(X0)
        self.meas_dim = len(meas_cov)
        self.Dt = Dt
        self.proc_cov = proc_cov
        self.meas_cov = meas_cov
        self.est_cov = copy.deepcopy(proc_cov)
        if Hk != None:
            self.Hk = Hk
        self.L = L
        self.index = 0
        self.t = 0
        self.tvec = zeros((self.L+1, 1))
        self.xfilt = zeros((self.L+1, self.dim))
        self.xpred = zeros((self.L+1, self.dim))
        self.covs = zeros((self.L+1, self.dim, self.dim))
        self.xfilt[0] = X0
        self.xpred[0] = X0
        self.covs[0] = self.est_cov
        self.measurements = zeros((self.L+1, self.meas_dim))
        self.measurements[0] = tools.matmult(self.Hk, self.X0)
        self.xkk = X0
        self.Qk = self.proc_cov
        self.Rk = self.meas_cov
        self.Pkk = copy.deepcopy(self.est_cov)


    def step_filter(self, zt, Winc=None, *args, **kwargs):
        if Winc == None:
            Winc = random.multivariate_normal(zeros(self.dim),
                                              self.proc_cov)
        self.index += 1
        self.t += self.Dt
        # prediction
        xkkm1 = self.state_trans(Winc, args, kwargs)
        Fkm1 = self.proc_linearize(xkkm1)
        Pkkm1 = tools.matmult(Fkm1,self.Pkk,Fkm1.T) + self.Qk
        # update
        yk = zt - tools.matmult(self.Hk, xkkm1)
        Sk = tools.matmult(self.Hk, Pkkm1, self.Hk.T) + self.Rk
        Kk = tools.matmult(Pkkm1, self.Hk.T, linalg.inv(Sk))
        self.xkk = xkkm1 + tools.matmult(Kk, yk)
        self.Pkk = tools.matmult((eye(self.dim) -
                                  tools.matmult(Kk,self.Hk)), Pkkm1)
        # store data
        self.xfilt[self.index] = self.xkk
        self.xpred[self.index] = xkkm1
        self.covs[self.index] = self.Pkk
        self.tvec[self.index] = self.t
        self.measurements[self.index] = zt
        return
    
    def store_data(self, fname):
        out = {}
        for name,var in self.__dict__.iteritems():
            if (sio.matlab.mio5.to_writeable(var) != None) and \
                (array(var).dtype.kind != 'O'):
                try:
                    out[name] = array(var)
                except TypeError:
                    print "Could not convert",name,"to array"
                    pass
        if os.path.splitext(fname)[1] != '.mat':
            fname += '.mat'
        sio.savemat(fname, out, appendmat=False, oned_as='row')
        return


    def get_attr_hash(self, num=8):
        hashstring = "%f%f%f%f%f%f%f%f%f" % (sum(self.X0), self.num, self.dim,
                                               self.Dt, sum(self.proc_cov),
                                               sum(self.meas_cov), sum(self.Hk),
                                               self.L, self.index)
        return hashlib.sha1(hashstring).hexdigest()[:num]


class VI_EKF( EKF ):
    """
    Inherits from EKF class.  Takes same args plus a trep system as kwarg.
    """
    def __init__(self, *args, **kwargs):
        self.sys = kwargs.pop('sys', None)
        if self.sys == None:
            print "Must give trep system as kwarg!"
            return
        super(VI_EKF, self).__init__(*args, **kwargs)
        self.mvi = trep.MidpointVI(self.sys)
        self.dsys = discopt.DSystem(self.mvi, array([0, self.Dt]))

    def state_trans(self, Winc, *args, **kwargs):
        self.dsys.set(self.xkk, uk=array([]), k=0)
        xkkm1 = self.dsys.f()
        self.Flin = self.dsys.fdx()
        return xkkm1

    def proc_linearize(self, xlin):
        return self.Flin


class EM_EKF( EKF ):
    """
    Inherits from EKF class, takes same args, plus an A matrix as a kwarg
    """
    def __init__(self, *args, **kwargs):
        self.A = kwargs.pop('A', None)
        if self.A == None:
            print "Must give A matrix as kwarg!"
            return
        super(EM_EKF, self).__init__(*args, **kwargs)

    def state_trans(self, Winc, *args, **kwargs):
        return self.xkk + self.Dt*tools.matmult(self.A, self.xkk)

    def proc_linearize(self, xlin):
        return self.A


class Nonlinear_EM_EKF( EKF ):
    """
    Inherits from EKF class, takes same args, plus an A matrix as a kwarg
    """
    def __init__(self, *args, **kwargs):
        self.A = kwargs.pop('A', None)
        if self.A == None:
            print "Must give A as kwarg!"
            return
        self.f = kwargs.pop('f', None)
        if self.f == None:
            print "Must give f callable as kwarg!"
            return
       
        super(Nonlinear_EM_EKF, self).__init__(*args, **kwargs)

    def state_trans(self, Winc, *args, **kwargs):
        return tools.nonlin_state_trans_euler(self.f, self.xkk, self.Dt, zeros(self.xkk.shape))

    def proc_linearize(self, xlin):
        return eye(self.dim) + self.Dt*self.A(xlin)


class Nonlinear_RK2_EKF( EKF ):
    """
    Inherits from EKF class, takes same args, plus an A matrix as a kwarg
    """
    def __init__(self, *args, **kwargs):
        self.A = kwargs.pop('A', None)
        if self.A == None:
            print "Must give A callable as kwarg!"
            return
        self.f = kwargs.pop('f', None)
        if self.f == None:
            print "Must give f callable as kwarg!"
            return
       
        super(Nonlinear_RK2_EKF, self).__init__(*args, **kwargs)

    def state_trans(self, Winc, *args, **kwargs):
        return tools.nonlin_state_trans_rk2(self.f, self.xkk, self.Dt, zeros(self.xkk.shape))

    def proc_linearize(self, xlin):
        return (eye(self.dim) + tools.matmult(
            self.Dt*self.A(xlin + 0.5*self.Dt*self.f(xlin)),
            eye(self.dim) + 0.5*self.Dt*self.A(xlin)))
    
