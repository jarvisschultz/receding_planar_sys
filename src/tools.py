from numpy import (
    dot, array, pi, exp, sqrt, apply_along_axis, zeros
    )
import rospy
import system_definition as sd
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.msg import PlanarSystemState


def matmult(*x):
    """
    Shortcut for standard matrix multiplication.
    matmult(A,B,C) returns A*B*C.
    """
    return reduce(dot, x)


def state_trans_dist_euler_single_op(X, A, dt, noise):
    fx = array([tools.matmult(A,x) for x in X])
    return X + dt*fx + noise


def nonlin_state_trans_euler(f, X, dt, noise):
    return dt*apply_along_axis(f, X.ndim-1, X) + X + noise


def nonlin_state_trans_rk2(f, X, dt, noise):
    k1 = dt*apply_along_axis(f, X.ndim-1, X)
    k2 = dt*apply_along_axis(f, X.ndim-1, X + 0.5*k1)
    return X + k2 + noise


def linear_norm_cpdf_one_d_single_op(z, X, H, cov):
    tmp = [dot(H,x) for x in X]
    return (1.0/sqrt(2.0*pi*cov)*exp(-(z-tmp)**2.0/(2.0*cov)))


####################################################################
# FUNCTIONS FOR CONVERTING ROS MESSAGES INTO NUMPY ARRAYS AND BACK #
####################################################################
def state_to_array(system, state):
    """
    Convert a PlanarSystemState message to an array of appropriate length
    and in the correct order
    """
    n = system.nQ
    out = zeros(2*n)
    out[system.get_config('xm').index] = state.xm
    out[system.get_config('ym').index] = state.ym
    out[system.get_config('xr').index] = state.xr
    out[system.get_config('r').index] = state.r
    out[system.get_config('xm').index + n] = state.pxm
    out[system.get_config('ym').index + n] = state.pym
    out[system.get_config('xr').index + n] = state.vxr
    out[system.get_config('r').index + n] = state.vr
    return out


def config_to_array(system, config):
    """
    Convert a PlanarSystemConfig message to an array of appropriate length
    and in the correct order
    """
    n = system.nQ
    out = zeros(n)
    out[system.get_config('xm').index] = config.xm
    out[system.get_config('ym').index] = config.ym
    out[system.get_config('xr').index] = config.xr
    out[system.get_config('r').index] = config.r
    return out


def array_to_config(system, config, configMsg):
    """
    Convert a numpy array (config) into a PlanarSystemConfig message (configMsg)
    """
    for q in system.configs:
        configMsg.__setattr__(q.name, config[q.index])
    return


def array_to_state(system, state, stateMsg):
    """
    Convert a numpy array (state) into a PlanarSystemState message (stateMsg)
    """
    for q in system.configs:
        stateMsg.__setattr__(q.name, state[q.index])
        if q.kinematic: name = 'v' + q.name
        else: name = 'p' + q.name
        stateMsg.__setattr__(name, state[q.index + system.nQ])
    return
    
