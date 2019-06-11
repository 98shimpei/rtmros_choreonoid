#!/usr/bin/env python

from hrpsys_choreonoid_tutorials.choreonoid_hrpsys_config_hrp2 import *

class HRP2JSKNTS_HrpsysConfigurator(ChoreonoidHrpsysConfigurator):
    def getRTCList (self):
        ##return self.getRTCListUnstable()
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            ['vs', "VirtualForceSensor"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['es', "EmergencyStopper"],
            ['ic', "ImpedanceController"],
            ['abc', "AutoBalancer"],
            ['st', "Stabilizer"],
            ['co', "CollisionDetector"],
            # ['tc', "TorqueController"],
            # ['te', "ThermoEstimator"],
            # ['tl', "ThermoLimiter"],
            ['rfu', "ReferenceForceUpdater"],
            ['octd', "ObjectContactTurnaroundDetector"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]

    # def defJointGroups (self):
    #     rarm_group = ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7']]
    #     larm_group = ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7']]
    #     rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5', 'RLEG_JOINT6']]
    #     lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5', 'LLEG_JOINT6']]
    #     head_group = ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']]
    #     torso_group = ['torso', ['CHEST_JOINT0', 'CHEST_JOINT1']]
    #     self.Groups = [rarm_group, larm_group, rleg_group, lleg_group, head_group, torso_group]

    def startABSTIMP (self):
        ### not used on hrpsys
        self.el_svc.setServoErrorLimit("RARM_JOINT7", sys.float_info.max)
        self.el_svc.setServoErrorLimit("LARM_JOINT7", sys.float_info.max)
        self.rh_svc.servo("RARM_JOINT7",OpenHRP.RobotHardwareService.SWITCH_OFF)
        self.rh_svc.servo("LARM_JOINT7",OpenHRP.RobotHardwareService.SWITCH_OFF)
        self.rh_svc.setServoGainPercentage("RLEG_JOINT6", 30.0)
        self.rh_svc.setServoGainPercentage("LLEG_JOINT6", 30.0)
        self.seq_svc.setJointAngles(self.hrp2ResetPose(), 1.0)
        self.seq_svc.waitInterpolation()
        ###
        self.startAutoBalancer()
        # Suppress limit over message and behave like real robot that always angle-vector is in seq.
        # Latter four 0.0 are for hands.
        self.ic_svc.startImpedanceController("larm")
        self.ic_svc.startImpedanceController("rarm")
        self.startStabilizer()

if __name__ == '__main__':
    hcf = HRP2JSKNTS_HrpsysConfigurator("HRP2JSKNTS")
    [sys.argv, connect_constraint_force_logger_ports] = hcf.parse_arg_for_connect_ports(sys.argv)
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1], connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
        hcf.startABSTIMP()
    else :
        hcf.init(connect_constraint_force_logger_ports=connect_constraint_force_logger_ports)
