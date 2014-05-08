import time
import trep
import trep.discopt as discopt
import numpy as np
from collections import deque
import rospy
from tools import matmult as MM


def calc_initial_guess(dsys, X0, Xref, Uref):
    """
    Builds an initial trajectory guess using an initial condition, and a
    feedforward U
    """
    X,U = dsys.build_trajectory()
    X[0] = X0
    U = Uref
    for k in range(dsys.kf()):
        if k == 0:
            dsys.set(X[k], U[k], 0)
        else:
            dsys.step(U[k])
        X[k+1] = dsys.f()
    return X,U


def LQROptimizer(system, X0, DT, Q=None, R=None, tol=1e-6, steps=1000):
    """
    This class takes a system, an equilibrium state, a timestep, weighting
    matrices, an error tolerance. It then solves a discrete-time LQR problem
    to build a feedback regulator about the equilibrium point.
    """
    # first let's build an MVI and dsystem
    mvi = trep.MidpointVI(system)
    tvec = np.arange(0.0, steps*DT, DT)#np.array([0, DT])
    dsys = discopt.DSystem(mvi, tvec)

    # build reference trajectory:
    Xd = np.array([X0]*len(tvec))
    Ud = np.array([X0[system.nQd:system.nQ]]*(len(tvec)-1))

    # build cost matrices:
    if Q is None: Q = np.identity(dsys.nX)
    if R is None: R = np.identity(dsys.nU)
    Qk = lambda k: Q
    Rk = lambda k: R

    (Kstab, A, B) = dsys.calc_feedback_controller(Xd, Ud, Qk, Rk, return_linearization=True)
    err = False
    if np.linalg.norm(Kstab[0] - Kstab[1], ord=2) > tol:
        rospy.logwarn("LQR stabilizing controller not settled!")
        err = True
    return err, Kstab[0]


def DiscreteFiniteHorizonLQR(system, X0, DT, Q=None, R=None, tol=1e-6, steps=1000, Pk=None):
    mvi = trep.MidpointVI(system)
    dsys = discopt.DSystem(mvi, [0, DT])
    # create an A and B:
    Xt = np.vstack((X0,X0))
    Ut = np.array([X0[2:4]])
    A,B = dsys.linearize_trajectory(Xt, Ut)
    A = A[0]
    B = B[0]
    # now we can start solving Riccati equation:
    if Pk is None:
        Pk = Q
    Pkp1 = 1000*Pk
    err = True
    for i in range(steps):
        Pkp1 = Q + MM(A.T, Pk, A) - MM(A.T, Pk, B, np.linalg.inv((R+MM(B.T,Pk,B))),B.T,Pk,A)
        if np.linalg.norm(Pkp1 - Pk, ord=2) < tol:
            err = False
            break
        Pk = Pkp1
    # calc K
    K = MM(np.linalg.inv(R+MM(B.T,Pk,B)), B.T, Pk, A)
    return err, K, Pk

    
class RecedingOptimizer( object ):

    def __init__(self, system, t, beta=0.7, tolerance=1e-1, DT=None):
        # create a VI
        self.mvi = trep.MidpointVI(system)
        # create a dsys object:
        self.dsys = discopt.DSystem(self.mvi, t)
        # set default optimization props:
        self.beta = beta
        self.tolerance = tolerance
        if DT:
            self.DT = DT
        else:
            self.DT = t[1]-t[0]
        # setup parameters for estimating whether we have time to take a step:
        N = 3
        self.tsamps = deque([self.DT/3.0]*N, maxlen=N)
        self.coeffs = [2, 4, 10]


    def optimize_window(self, Qcost, Rcost, Xref, Uref, X0, U0):
        # build cost object:
        cost = discopt.DCost(Xref, Uref, Qcost, Rcost)
        # build optimizer:
        optimizer = discopt.DOptimizer(self.dsys, cost)
        optimizer.optimize_ic = False
        optimizer.descent_tolerance = self.tolerance
        optimizer.first_method_iterations = 0
        optimizer.armijo_beta = self.beta
        finished = False
        optimizer.monitor = discopt.DOptimizerMonitor()
        error = False
        try:
            tstart = time.time()
            step_count = 0
            first = True
            while not finished:
                tstep = time.time()
                # do we have time to take step?
                if (time.time()-tstart) + \
                    np.average(self.tsamps, weights=self.coeffs) > self.DT:
                    rospy.logwarn("Optimization exited early " \
                                  "after {0:d} steps".format(step_count))
                    break
                if first: method='steepest'
                else: method='newton'
                first = False
                finished,X0,U0,dcost,ncost = optimizer.step(
                    step_count, X0, U0, method=method)
                self.tsamps.append(time.time()-tstep)
                step_count += 1
                if time.time() - tstart > self.DT:
                    rospy.logwarn("Time elapsed = "\
                      "{0:4.4f} s for {1:d} steps".format(time.time()-tstart,
                                                          step_count))
                    break
        except trep.ConvergenceError as e:
            rospy.loginfo("Detected optimization problem: %s"%e.message)
            error = True
        # except:
        #     rospy.logerr("Unknown error!")
        #     error = True
        optsum = {'cost': ncost,
                  'dcost': dcost,
                  'steps': step_count,
                  'tolerance': optimizer.descent_tolerance,
                  'error': error,
                  'done': finished}
        return optsum, X0, U0

