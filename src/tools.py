from numpy import (
    dot, array, pi, exp, sqrt, apply_along_axis
    )

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

