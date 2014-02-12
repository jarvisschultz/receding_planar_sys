import trep
from trep import tx, ty, tz, rx, ry, rz

## define some global constants:
BALL_MASS = 0.1244 ## kg
g = 9.81 ## m/s^2
h0 = 1 ## default height of robot in m
DT = 1/30.0 ## nominal dt for the system
CALLBACK_DIVISOR = 30


def MassSystem2D(self):
    # define system:
    system = trep.System()

    frames = [
        tx('xm', name='x-mass'), [
            ty('ym', name='y-mass', mass=BALL_MASS) ],
        ty(1, name='robot_plane'), [
            tx('xr', name='x-robot', kinematic=True) ]]
    system.import_frames(frames)
    trep.potentials.Gravity(system, (0, -g, 0))
    trep.forces.Damping(system, 0.05)

    # add string constraint as a kinematic configuration var
    trep.constraints.Distance(system, 'y-mass', 'x-robot','r')
    return system

