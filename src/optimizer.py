import time
import trep
import trep.discopt as discopt
import numpy as np
from collections import deque

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
        self.coeffs = [10, 4, 2]


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
                    print "[WARN] Step number {1:d} exited early after  "\
                      "{0:d} steps".format(step_count, i)
                    break
                if first: method='steepest'
                else: method='newton'
                first = False
                finished,X0,U0,dcost,ncost = optimizer.step(
                    step_count, X0, U0, method=method)
                self.tsamps.append(time.time()-tstep)
                step_count += 1
                if time.time() - tstart > self.DT:
                    print "[WARN] Step number {2:d} time elapsed = "\
                      "{0:4.4f} s for {1:d} steps".format(time.time()-tstart,
                                                          step_count, i)
                    break
        except trep.ConvergenceError as e:
            print("Detected optimization problem: %s"%e.message)
            error = True
        except:
            print "[ERROR] Unknown error!"
            error = True
        return error, X0, U0

