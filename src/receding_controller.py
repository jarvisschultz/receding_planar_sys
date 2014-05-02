#!/usr/bin/env python
"""
Jarvis Schultz
Feb 2014

This node runs a true receding horizon optimization.


SUBSCRIPTIONS:
    - meas_config (PlanarSystemConfig)
    - /operating_condition (OperatingCondition)

PUBLISHERS:
    - filt_config (PlanarSystemConfig)
    - ref_config (PlanarSystemConfig)
    - serial_commands (RobotCommands)
    - mass_ref_path (Path)
    - mass_filt_path (Path)
    - post_covariance (PlanarCovariance)
    - filt_state (PlanarSystemState)
    - ref_state (PlanarSystemState)
    - start_time (Time)
    - optimization_data (OptimizationData)

SERVICES:
    - get_ref_config (PlanarSystemService) (provider)
    - operating_condition_change (OperatingConditionChange) (client)

PARAMS:
    - robot_index ~ which robot to communicate with
    - controller_freq ~ defines the rate that everything is happening at
    - window_length ~ defines number of timesteps in an optimization
    - time_final ~ defines the total time to run the controller
    - ~tper ~ how fast to run through reference traj
    - ~rx ~ half-width of reference traj
    - ~ry ~ half-height of reference traj
    - ~power ~ power of superellipse
    - ~r0 ~ inverse of exponential lead-in time constant
    - ~q_weight ~ diagonal of Q in optimizations
    - ~r_weight ~ diagonal of R in optimizations
    - ~meas_cov ~ diagonal of measurement covariance
    - ~proc_cov ~ diagonal of process covariance
"""

# ROS imports:
import roslib; roslib.load_manifest('receding_planar_sys')
import rospy
import tf
from std_msgs.msg import Time
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.msg import RobotCommands
from puppeteer_msgs.msg import PlanarCovariance
from puppeteer_msgs.msg import PlanarSystemState
from puppeteer_msgs.msg import OperatingCondition
from puppeteer_msgs.msg import OptimizationData
from puppeteer_msgs.srv import PlanarSystemService
from puppeteer_msgs.srv import PlanarSystemServiceRequest
from puppeteer_msgs.srv import OperatingConditionChange
from puppeteer_msgs.srv import OperatingConditionChangeRequest
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# misc imports:
import numpy as np
from collections import deque
import copy

# local imports
import optimizer as op
import reference_manager as rm
import system_definition as sd
import ekf
import tools


# global constants:
DT = 1/10. # timestep
LEN = 20 # number of time steps to optimize over
PATH_TIME = 6.0 # number of seconds to store paths
TPER = 10.0 # time of ellipse traversal


class RecedingController:

    def __init__(self):
        rospy.loginfo("Creating RecedingController class...")

        # first let's get all necessary parameters:
        refargs = self.get_and_set_params()
        self.RM = rm.RefManager(**refargs)

        # create time vectors, system, vi, and dsys
        self.system = sd.MassSystem2D()
        self.mvi = sd.trep.MidpointVI(self.system)
        self.tvec = np.arange(0, self.dt*(int(self.tf/self.dt)+1), self.dt)
        self.twin = np.arange(0, self.dt*self.n_win, self.dt)
        self.dsys = op.discopt.DSystem(self.mvi, self.twin)
        self.dsyssim = op.discopt.DSystem(self.mvi, [0, self.dt])
        
        # create optimizer and cost matrices
        self.optimizer = op.RecedingOptimizer(self.system, self.twin, DT=self.dt)
        if rospy.has_param("~q_weight"):
            Q = rospy.get_param("~q_weight")
            if len(Q) is not self.dsys.nX:
                rospy.logerr("q_weight parameter is wrong length!")
                rospy.signal_shutdown()
            self.Qcost = np.diag(Q)
        if rospy.has_param("~r_weight"):
            R = rospy.get_param("~r_weight")
            if len(R) is not self.dsys.nU:
                rospy.logerr("r_weight parameter is wrong length!")
                rospy.signal_shutdown()
            self.Rcost = np.diag(R)

        # get the initial config of the system:
        X,U = self.RM.calc_reference_traj(self.dsys, [0])
        self.X0 = X[0]
        self.Q0 = self.X0[0:self.system.nQ]

        # get full reference:
        self.Xref, self.Uref = self.RM.calc_reference_traj(self.dsys, self.tvec)
        
        # create EKF:
        if rospy.has_param("~meas_cov"):
            Q = rospy.get_param("~meas_cov")
            if len(Q) is not self.system.nQ:
                rospy.logerr("meas_cov parameter is wrong length!")
                rospy.signal_shutdown()
            self.meas_cov = np.diag(Q)
        if rospy.has_param("~proc_cov"):
            Q = rospy.get_param("~proc_cov")
            if len(Q) is not self.dsys.nX:
                rospy.logerr("proc_cov parameter is wrong length!")
                rospy.signal_shutdown()
            self.proc_cov = np.diag(Q)
        self.Hk = np.hstack((np.eye(self.system.nQ),
                             np.zeros((self.system.nQ, self.system.nQ))))
        self.ekf = ekf.VI_EKF('vi_ekf', self.X0, self.dt, self.proc_cov, self.meas_cov,
                              Hk=self.Hk, sys=self.system, L=len(self.tvec))

        # define some vars that we are going to need
        self.first_flag = True
        self.tbase = rospy.Time.now()
        self.callback_count = 0
        # self.Uprev = np.zeros(self.dsys.nU)
        Xtmp,Utmp = self.RM.calc_reference_traj(self.dsys, [0, self.dt])
        self.Ukey = Utmp[0]
        self.Uprev = Utmp[0]
        self.mass_ref_vec = deque([], maxlen=self.PATH_LENGTH)
        self.mass_filt_vec = deque([], maxlen=self.PATH_LENGTH)

        # create a service provider for the reference configuration
        self.config_serv = rospy.Service("get_ref_config", PlanarSystemService,
                                            self.ref_config_service_handler)
        # wait for service servers:
        rospy.loginfo("Waiting for operating_condition_change service")
        rospy.wait_for_service("/operating_condition_change")
        rospy.loginfo("operating_condition_change service now available")
        self.op_change_client = rospy.ServiceProxy("/operating_condition_change",
                                                   OperatingConditionChange)
        # idle on startup
        self.operating_condition = OperatingCondition.IDLE
        try:
            self.op_change_client(OperatingCondition(self.operating_condition))
        except rospy.ServiceException, e:
            rospy.loginfo("Service did not process request: %s"%str(e))
        # define subscribers:
        self.op_cond_sub = rospy.Subscriber("/operating_condition",
                                            OperatingCondition, self.opcb)
        if self.ctype == "openloop":
            rospy.loginfo("Running an open-loop, IK controller")
            self.meas_sub = rospy.Subscriber("meas_config", PlanarSystemConfig,
                                             self.openloopcb)
        elif self.ctype == "receding":
            rospy.loginfo("Running a receding horizon controller")
            self.meas_sub = rospy.Subscriber("meas_config", PlanarSystemConfig,
                                             self.recedingcb)
        elif self.ctype == "lqr":
            rospy.loginfo("Running an LQR controller")
            self.setup_lqr_controller()
            self.meas_sub = rospy.Subscriber("meas_config", PlanarSystemConfig,
                                             self.lqrcb)
        elif self.ctype == "full":
            rospy.loginfo("Running a full trajectory optimization")
            self.setup_full_controller()
            self.meas_sub = rospy.Subscriber("meas_config", PlanarSystemConfig,
                                             self.fullcb)
        elif self.ctype == "feedforward":
            rospy.loginfo("Running a feedforward only optimization")
            self.setup_full_controller()
            self.meas_sub = rospy.Subscriber("meas_config", PlanarSystemConfig,
                                             self.feedforwardcb)
        else:
            rospy.logwarn("Unrecognized controller type, falling back to receding")
            

        # define publishers:
        self.time_pub = rospy.Publisher("start_time", Time)
        self.filt_pub = rospy.Publisher("filt_config", PlanarSystemConfig)
        self.filt_state_pub = rospy.Publisher("filt_state", PlanarSystemState)
        self.ref_pub = rospy.Publisher("ref_config", PlanarSystemConfig)
        self.ref_state_pub = rospy.Publisher("ref_state", PlanarSystemState)
        self.comm_pub = rospy.Publisher("serial_commands", RobotCommands)
        self.ref_path_pub = rospy.Publisher("mass_ref_path", Path)
        self.filt_path_pub = rospy.Publisher("mass_filt_path", Path)
        self.cov_pub = rospy.Publisher("post_covariance", PlanarCovariance)
        self.opt_pub = rospy.Publisher("optimization_data", OptimizationData)
        # define timer callbacks:
        self.path_timer = rospy.Timer(rospy.Duration(0.1), self.path_timercb)

        # send the robot it's starting pose in the /optimization_frame
        rospy.logwarn("Waiting for three seconds!!!")
        rospy.sleep(3)
        rospy.loginfo("Ready to go!!!")
        self.send_initial_config()
        # send robot a start command:
        self.send_start_command()

        return
        


    def ref_config_service_handler(self, req):
        if req.index is not 0:
            rospy.logerr("Current provider cannot handle index request")
            return None
        elif req.t < 0.0 or req.t > self.tf:
            rospy.logerr("Requested time %f is outside of valid horizon", req.t)
            return None
        # get X,U at requested time
        X,U = self.RM.calc_reference_traj(self.dsys, [req.t])
        # fill out message
        config = PlanarSystemConfig()
        config.xm = X[0,self.system.get_config('xm').index]
        config.ym = X[0,self.system.get_config('ym').index]
        config.xr = X[0,self.system.get_config('xr').index]
        config.r = X[0,self.system.get_config('r').index]
        config.header.frame_id = "/optimization_frame"
        config.header.stamp = rospy.Time.now()
        time = req.t
        length = len(self.tvec)
        xtmp = X[0]
        dt = self.dt
        return {'config' : config,
                'state' : xtmp,
                'dt' : dt,
                'length' : length,
                'time' : time,
                }


    def setup_lqr_controller(self):
        steps = 500
        err = True
        count = 0
        while err:
            err, Kstab = op.LQROptimizer(self.system, self.X0, self.dt,
                                         Q=self.Qcost, R=self.Rcost, steps=steps)
            count += 1
            steps *= 2
            if count > 5:
                rospy.logerr("Could not find LQR regulator!")
                rospy.signal_shutdown()
        self.Kstab = Kstab
        return


    def setup_full_controller(self):
        # create mvi and dsys:
        mvi = sd.trep.MidpointVI(self.system)
        dsys = op.discopt.DSystem(mvi, self.tvec)
        # get reference and initial guess:
        X0,U0 = op.calc_initial_guess(dsys, self.X0, self.Xref, self.Uref)
        cost = op.discopt.DCost(self.Xref, self.Uref, self.Qcost, self.Rcost)
        # build optimizer:
        optimizer = op.discopt.DOptimizer(dsys, cost)
        optimizer.optimize_ic = False
        optimizer.descent_tolerance = 1e-6
        optimizer.first_method_iterations = 2
        # optimize:
        finished, self.Xopt, self.Uopt = optimizer.optimize(X0, U0)
        Qk = lambda k: self.Qcost
        Rk = lambda k: self.Rcost
        self.Kstab = dsys.calc_feedback_controller(self.Xopt, self.Uopt, Qk, Rk)
        if not finished:
            rospy.logwarn("Could not complete full optimization!")
        return


    def opcb(self, data):
        # rospy.loginfo("Operating condition cb: %d"%data.state)
        self.operating_condition = data.state
        return


    def recedingcb(self, data):
        rospy.logdebug("receding control callback triggered")
        # op_cond = rospy.get_param("/operating_condition")
        op_cond = self.operating_condition
        if op_cond is OperatingCondition.CALIBRATE:
            self.send_initial_config()
            # self.first_flag = False
            return
        elif op_cond is OperatingCondition.EMERGENCY:
            # we need to stop robots!:
            self.stop_robots()
            return
        elif op_cond is not OperatingCondition.RUN:
            # we are not running, keep setting initializations
            self.first_flag = True
            self.callback_count = 0
            self.ekf.index = 0
            # clear trajectories:
            self.mass_ref_vec.clear()
            self.mass_filt_vec.clear()
            return

        if self.first_flag:
            rospy.loginfo("Beginning Trajectory")
            # publish start time:
            self.time_pub.publish(data.header.stamp)
            self.send_initial_config()
            self.tbase = data.header.stamp
            self.first_flag = False
            self.callback_count = 0
            # reset filter:
            self.ekf.index = 0
            self.ekf.xkk = self.ekf.X0
            self.ekf.est_cov = copy.deepcopy(self.ekf.proc_cov)
            # get reference traj after initial dt:
            Xtmp,Utmp = self.RM.calc_reference_traj(self.dsys, [0, self.dt, 2*self.dt])
            # send reference traj U and store:
            self.convert_and_send_input(Xtmp[0][2:4], Xtmp[1][2:4])#self.Uprev, self.Ukey)
            self.Uprev = Utmp[0]
            self.Ukey = Utmp[1]
            # publish filtered and reference:
            self.publish_state_and_config(data, self.ekf.xkk, self.X0)
        else:
            self.callback_count += 1
            zk = tools.config_to_array(self.system, data)
            # get updated estimate
            self.ekf.step_filter(zk, Winc=np.zeros(self.dsys.nX), u=self.Uprev)
            # send old value to robot:
            self.convert_and_send_input(self.ekf.xkk[2:4], self.Ukey)
            # publish filtered and reference:
            xref,uref = self.RM.calc_reference_traj(self.dsys, [self.callback_count*self.dt])
            self.publish_state_and_config(data, self.ekf.xkk, xref[0])
            # get prediction of where we will be in +dt seconds
            self.dsyssim.set(self.ekf.xkk, self.Ukey, 0)
            Xstart = self.dsyssim.f()
            # get reference traj
            ttmp = self.twin + (self.callback_count + 1)*self.dt
            Xref, Uref = self.RM.calc_reference_traj(self.dsys, ttmp)
            # get initial guess
            X0, U0 = op.calc_initial_guess(self.dsys, Xstart, Xref, Uref)
            # add path information:
            self.add_to_path_vectors(data, Xref[0], Xstart)
            # optimize
            optdat,X,U =  self.optimizer.optimize_window(self.Qcost, self.Rcost,
                                                        Xref, Uref, X0, U0)
            if optdat['error']:
                rospy.logwarn("Received an error from optimizer!")
            # publish optimization summary:
            od = OptimizationData(**optdat)
            od.index = self.callback_count
            od.time = ttmp[0]
            self.opt_pub.publish(od)
            # store data:
            self.Uprev = self.Ukey
            self.Ukey = U[0]
            # check for the end of the trajectory:
            if self.callback_count >= len(self.tvec):
                rospy.loginfo("Trajectory complete!")
                try:
                    self.op_change_client(OperatingCondition(OperatingCondition.STOP))
                except rospy.ServiceException, e:
                    rospy.loginfo("Service did not process request: %s"%str(e))
                # now we can stop the robots
                self.stop_robots()
        return


    def lqrcb(self, data):
        rospy.logdebug("LQR control callback triggered")
        op_cond = self.operating_condition
        if op_cond is OperatingCondition.CALIBRATE:
            self.send_initial_config()
            # self.first_flag = False
            return
        elif op_cond is OperatingCondition.EMERGENCY:
            # we need to stop robots!:
            self.stop_robots()
            return
        elif op_cond is not OperatingCondition.RUN:
            # we are not running, keep setting initializations
            self.first_flag = True
            self.callback_count = 0
            self.ekf.index = 0
            # clear trajectories:
            self.mass_ref_vec.clear()
            self.mass_filt_vec.clear()
            return

        if self.first_flag:
            rospy.loginfo("Beginning Trajectory")
            # publish start time:
            self.time_pub.publish(data.header.stamp)
            self.send_initial_config()
            self.tbase = data.header.stamp
            self.first_flag = False
            self.callback_count = 0
            # reset filter:
            self.ekf.index = 0
            self.ekf.xkk = self.ekf.X0
            self.ekf.est_cov = copy.deepcopy(self.ekf.proc_cov)
            # get reference traj after initial dt:
            Xtmp,Utmp = self.RM.calc_reference_traj(self.dsys, [0, self.dt, 2*self.dt])
            # send reference traj U and store:
            self.convert_and_send_input(Xtmp[0][2:4], Xtmp[1][2:4])#self.Uprev, self.Ukey)
            self.Uprev = Utmp[0]
            self.Ukey = Utmp[1]
            # publish filtered and reference:
            self.publish_state_and_config(data, self.ekf.xkk, self.X0)
        else:
            self.callback_count += 1
            zk = tools.config_to_array(self.system, data)
            # get updated estimate
            self.ekf.step_filter(zk, Winc=np.zeros(self.dsys.nX), u=self.Uprev)
            # get reference
            xref,uref = self.RM.calc_reference_traj(self.dsys, [self.callback_count*self.dt])
            # publish filtered and reference:
            self.publish_state_and_config(data, self.ekf.xkk, xref[0])
            # calculate controls:
            self.Ukey = xref[0][2:4] + tools.matmult(self.Kstab, xref[0] - self.ekf.xkk)
            # send controls to robot:
            self.convert_and_send_input(self.ekf.xkk[2:4], self.Ukey)
            # add path information:
            self.add_to_path_vectors(data, xref[0], self.ekf.xkk)
            # store data:
            self.Uprev = self.Ukey
            # check for the end of the trajectory:
            if self.callback_count >= len(self.tvec):
                rospy.loginfo("Trajectory complete!")
                try:
                    self.op_change_client(OperatingCondition(OperatingCondition.STOP))
                except rospy.ServiceException, e:
                    rospy.loginfo("Service did not process request: %s"%str(e))
                # now we can stop the robots
                self.stop_robots()
        return



    def fullcb(self, data):
        rospy.logdebug("Full controller callback triggered")
        op_cond = self.operating_condition
        if op_cond is OperatingCondition.CALIBRATE:
            self.send_initial_config()
            # self.first_flag = False
            return
        elif op_cond is OperatingCondition.EMERGENCY:
            # we need to stop robots!:
            self.stop_robots()
            return
        elif op_cond is not OperatingCondition.RUN:
            # we are not running, keep setting initializations
            self.first_flag = True
            self.callback_count = 0
            self.ekf.index = 0
            # clear trajectories:
            self.mass_ref_vec.clear()
            self.mass_filt_vec.clear()
            return

        if self.first_flag:
            rospy.loginfo("Beginning Trajectory")
            # publish start time:
            self.time_pub.publish(data.header.stamp)
            self.send_initial_config()
            self.tbase = data.header.stamp
            self.first_flag = False
            self.callback_count = 0
            # reset filter:
            self.ekf.index = 0
            self.ekf.xkk = self.ekf.X0
            self.ekf.est_cov = copy.deepcopy(self.ekf.proc_cov)
            # send reference traj U and store:
            self.Uprev = self.X0[2:4]
            self.Ukey = self.Uopt[0]
            # publish filtered and reference:
            self.publish_state_and_config(data, self.ekf.xkk, self.X0)
        else:
            self.callback_count += 1
            zk = tools.config_to_array(self.system, data)
            # get updated estimates
            self.ekf.step_filter(zk, Winc=np.zeros(self.dsys.nX), u=self.Uprev)
            # set index value:
            if self.callback_count > len(self.Uopt)-1:
                index = len(self.Uopt)-1
            else:
                index = self.callback_count
            # publish filtered and reference:
            self.publish_state_and_config(data, self.ekf.xkk, self.Xref[index])
            # calculate controls:
            self.Ukey = self.Uopt[index] + \
                        tools.matmult(self.Kstab[index], self.Xopt[index] - self.ekf.xkk)
            # send controls to robot:
            self.convert_and_send_input(self.ekf.xkk[2:4], self.Ukey)
            # add path information:
            self.add_to_path_vectors(data, self.Xref[index], self.ekf.xkk)
            # store data:
            self.Uprev = self.Ukey
            # check for the end of the trajectory:
            if self.callback_count > len(self.tvec)-1:
                rospy.loginfo("Trajectory complete!")
                try:
                    self.op_change_client(OperatingCondition(OperatingCondition.STOP))
                except rospy.ServiceException, e:
                    rospy.loginfo("Service did not process request: %s"%str(e))
                # now we can stop the robots
                self.stop_robots()
        return


    def feedforwardcb(self, data):
        rospy.logdebug("Feedforward controller callback triggered")
        op_cond = self.operating_condition
        if op_cond is OperatingCondition.CALIBRATE:
            self.send_initial_config()
            # self.first_flag = False
            return
        elif op_cond is OperatingCondition.EMERGENCY:
            # we need to stop robots!:
            self.stop_robots()
            return
        elif op_cond is not OperatingCondition.RUN:
            # we are not running, keep setting initializations
            self.first_flag = True
            self.callback_count = 0
            self.ekf.index = 0
            # clear trajectories:
            self.mass_ref_vec.clear()
            self.mass_filt_vec.clear()
            return

        if self.first_flag:
            rospy.loginfo("Beginning Trajectory")
            # publish start time:
            self.time_pub.publish(data.header.stamp)
            self.send_initial_config()
            self.tbase = data.header.stamp
            self.first_flag = False
            self.callback_count = 0
            # reset filter:
            self.ekf.index = 0
            self.ekf.xkk = self.ekf.X0
            self.ekf.est_cov = copy.deepcopy(self.ekf.proc_cov)
            # send reference traj U and store:
            self.Uprev = self.X0[2:4]
            self.Ukey = self.Uopt[0]
            # publish filtered and reference:
            self.publish_state_and_config(data, self.ekf.xkk, self.X0)
        else:
            self.callback_count += 1
            zk = tools.config_to_array(self.system, data)
            # get updated estimates
            self.ekf.step_filter(zk, Winc=np.zeros(self.dsys.nX), u=self.Uprev)
            # set index value:
            if self.callback_count > len(self.Uopt)-1:
                index = len(self.Uopt)-1
            else:
                index = self.callback_count
            # publish filtered and reference:
            self.publish_state_and_config(data, self.ekf.xkk, self.Xref[index])
            # calculate controls:
            self.Ukey = self.Uopt[index]
            # send controls to robot:
            self.convert_and_send_input(self.ekf.xkk[2:4], self.Ukey)
            # add path information:
            self.add_to_path_vectors(data, self.Xref[index], self.ekf.xkk)
            # store data:
            self.Uprev = self.Ukey
            # check for the end of the trajectory:
            if self.callback_count > len(self.tvec)-1:
                rospy.loginfo("Trajectory complete!")
                try:
                    self.op_change_client(OperatingCondition(OperatingCondition.STOP))
                except rospy.ServiceException, e:
                    rospy.loginfo("Service did not process request: %s"%str(e))
                # now we can stop the robots
                self.stop_robots()
        return



    def openloopcb(self, data):
        rospy.logdebug("openloop callback triggered")
        op_cond = self.operating_condition
        if op_cond is OperatingCondition.CALIBRATE:
            self.send_initial_config()
            # self.first_flag = False
            return
        elif op_cond is OperatingCondition.EMERGENCY:
            # we need to stop robots!:
            self.stop_robots()
            return
        elif op_cond is not OperatingCondition.RUN:
            # we are not running, keep setting initializations
            self.first_flag = True
            self.callback_count = 0
            self.ekf.index = 0
            # clear trajectories:
            self.mass_ref_vec.clear()
            self.mass_filt_vec.clear()
            return

        if self.first_flag:
            rospy.loginfo("Beginning Trajectory")
            # publish start time:
            self.time_pub.publish(data.header.stamp)
            self.send_initial_config()
            self.tbase = data.header.stamp
            self.first_flag = False
            self.callback_count = 0
            # reset filter:
            self.ekf.index = 0
            self.ekf.xkk = self.ekf.X0
            self.ekf.est_cov = copy.deepcopy(self.ekf.proc_cov)
            # get reference traj after initial dt:
            Xtmp,Utmp = self.RM.calc_reference_traj(self.dsys, [0, self.dt, 2*self.dt])
            # send reference traj U and store:
            self.convert_and_send_input(Xtmp[0][2:4], Xtmp[1][2:4])
            self.Uprev = Utmp[0]
            self.Ukey = Utmp[1]
            # publish filtered and reference:
            self.publish_state_and_config(data, self.ekf.xkk, self.X0)
        else:
            self.callback_count += 1
            zk = tools.config_to_array(self.system, data)
            # get updated estimate
            self.ekf.step_filter(zk, Winc=np.zeros(self.dsys.nX), u=self.Uprev)
            # send old value to robot:
            self.convert_and_send_input(self.ekf.xkk[2:4], self.Ukey)
            # publish filtered and reference:
            xref,uref = self.RM.calc_reference_traj(self.dsys, [self.callback_count*self.dt])
            self.publish_state_and_config(data, self.ekf.xkk, xref[0])
            # get prediction of where we will be in +dt seconds
            self.dsyssim.set(self.ekf.xkk, self.Ukey, 0)
            Xstart = self.dsyssim.f()
            # get reference traj
            ttmp = self.twin + (self.callback_count + 1)*self.dt
            Xref, Uref = self.RM.calc_reference_traj(self.dsys, ttmp)
            # get initial guess
            X0, U0 = op.calc_initial_guess(self.dsys, Xstart, Xref, Uref)
            # add path information:
            self.add_to_path_vectors(data, Xref[0], Xstart)
            # store data:
            self.Uprev = self.Ukey
            self.Ukey = U0[0]
            # check for the end of the trajectory:
            if self.callback_count >= len(self.tvec):
                rospy.loginfo("Trajectory complete!")
                try:
                    self.op_change_client(OperatingCondition(OperatingCondition.STOP))
                except rospy.ServiceException, e:
                    rospy.loginfo("Service did not process request: %s"%str(e))
                # now we can stop the robots
                self.stop_robots()
        return
    
    def get_and_set_params(self):
        # robot index:
        if rospy.has_param("robot_index"):
            tmp = rospy.get_param("robot_index")
            self.robot_index = tmp
        else:
            rospy.logwarn("Choosing default index for robot!")
            self.robot_index = 1
            rospy.set_param("robot_index", self.robot_index)
        # controller frequency
        if rospy.has_param("controller_freq"):
            tmp = rospy.get_param("controller_freq")
            self.dt = 1/float(tmp)
        else:
            tmp = 1/DT
            self.dt = DT
            rospy.set_param("controller_freq", tmp)
        rospy.loginfo("Controller frequency = %f Hz (dt = %f sec.)", tmp, self.dt)
        # window length
        if rospy.has_param("window_length"):
            tmp = rospy.get_param("window_length")
            self.n_win = int(tmp)
        else:
            self.n_win = LEN
            rospy.set_param("window_length", self.n_win)
        rospy.loginfo("Window Length: %d",self.n_win)
        # controller run time
        if rospy.has_param("time_final"):
            tmp = rospy.get_param("time_final")
            self.tf = float(tmp)
        else:
            self.tf = TPER
            rospy.set_param("time_final", self.tf)
        rospy.loginfo("Final Time: %d",self.tf)
        # set number of indices in path variables:
        self.PATH_LENGTH = int(PATH_TIME*1.0/self.dt)

        # what type of controller are we running
        if rospy.has_param("controller_type"):
            self.ctype = rospy.get_param("controller_type")
        else:
            self.ctype = "receding"
            rospy.set_param("controller_type", self.ctype)

        # get args for the reference manager:
        refargs = {}
        if rospy.has_param("~tper"):
            refargs['tper'] = rospy.get_param("~tper")
        if rospy.has_param("~rx"):
            refargs['rx'] = rospy.get_param("~rx")
        if rospy.has_param("~ry"):
            refargs['ry'] = rospy.get_param("~ry")
        if rospy.has_param("~power"):
            refargs['n'] = rospy.get_param("~power")
        if rospy.has_param("~r0"):
            refargs['r0'] = rospy.get_param("~r0")
        if rospy.has_param("~exponent"):
            tautmp = rospy.get_param("~exponent")
            if np.abs(tautmp) > 10e-6:
                refargs['tau'] = tautmp
        rospy.loginfo("REFERENCE PARAMETERS:")
        for key,val in refargs.iteritems():
            rospy.loginfo("\t{0:s} \t: {1:f}".format(key,val))
        return refargs


    def add_to_path_vectors(self, meas_data, ref_state, filt_state):
        pose = PoseStamped()
        pose.header.stamp = meas_data.header.stamp
        pose.header.frame_id = meas_data.header.frame_id
        # fill out filtered position:
        pose.pose.position.x = filt_state[self.system.get_config('xm').index]
        pose.pose.position.y = filt_state[self.system.get_config('ym').index]
        pose.pose.position.z = 0.0
        self.mass_filt_vec.append(pose)
        # fill out reference position and publish
        pose2 = copy.deepcopy(pose)
        pose2.pose.position.x = ref_state[self.system.get_config('xm').index]
        pose2.pose.position.y = ref_state[self.system.get_config('ym').index]
        pose2.pose.position.z = 0.0
        self.mass_ref_vec.append(pose2)
        return


    def publish_state_and_config(self, data, Xfilt, Xref):
        xmsg = PlanarSystemState()
        qmsg = PlanarSystemConfig()
        xmsg.header = data.header
        qmsg.header = data.header
        tools.array_to_state(self.system, Xfilt, xmsg)
        tools.array_to_config(self.system, Xfilt[0:self.system.nQ], qmsg)
        self.filt_state_pub.publish(xmsg)
        self.filt_pub.publish(qmsg) 
        # publish ref data:
        xmsg = PlanarSystemState()
        qmsg = PlanarSystemConfig()
        xmsg.header = data.header
        qmsg.header = data.header
        tools.array_to_state(self.system, Xref, xmsg)
        tools.array_to_config(self.system, Xref[0:self.system.nQ], qmsg)
        self.ref_state_pub.publish(xmsg)
        self.ref_pub.publish(qmsg)
        return

        
    def path_timercb(self, time_dat):
        if len(self.mass_ref_vec) > 0:
            # send reference path
            ref_path = Path()
            ref_path.header.stamp = time_dat.current_real
            ref_path.header.frame_id = self.mass_ref_vec[-1].header.frame_id
            ref_path.poses = list(self.mass_ref_vec)
            self.ref_path_pub.publish(ref_path)

        if len(self.mass_filt_vec) > 0:
            # send filtered path
            filt_path = Path()
            filt_path.header.stamp = time_dat.current_real
            filt_path.header.frame_id = self.mass_filt_vec[-1].header.frame_id
            filt_path.poses = list(self.mass_filt_vec)
            self.filt_path_pub.publish(filt_path)
        return
    

    def send_initial_config(self):
        com = RobotCommands()
        com.robot_index = self.robot_index
        com.type = ord('a')
        com.header.stamp = rospy.get_rostime()
        com.header.frame_id = "/optimization_frame"
        com.div = 4
        com.x = self.Q0[self.system.get_config('xr').index]
        com.y = 0
        com.th = 0
        com.height_left = self.Q0[self.system.get_config('r').index]
        com.height_right = 1
        self.comm_pub.publish(com)
        return


    def send_start_command(self):
        com = RobotCommands()
        com.robot_index = self.robot_index
        com.type = ord('m')
        com.header.stamp = rospy.get_rostime()
        self.comm_pub.publish(com)
        return


    def convert_and_send_input(self, u1, u2):
        """
        This function takes in two sets of kinematic inputs, processes them, and
        then sends them to the robot.
        u1 ~ where we think the kin configs are now
        u2 ~ where we want the kin configs to be in +dt seconds
        """
        ucom = (u2-u1)/self.dt
        com = RobotCommands()
        com.robot_index = self.robot_index
        com.type = ord('i')
        com.v_robot = ucom[0]
        com.w_robot = 0
        com.rdot = 0
        com.rdot_left = ucom[1]
        com.rdot_right = 0
        com.div = 3
        self.comm_pub.publish(com)
        return

    
    def stop_robots(self):
        com = RobotCommands()
        com.robot_index = self.robot_index
        com.type = ord('q')
        com.header.stamp = rospy.get_rostime()
        self.comm_pub.publish(com)
        return




def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('cl_control', log_level=rospy.INFO)

    try:
        controller = RecedingController()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
