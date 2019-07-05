#!/usr/bin/env python

from hrpsys_choreonoid_tutorials.choreonoid_hrpsys_config_hrp2 import *

class HRP2JSKNTS_HrpsysConfigurator(ChoreonoidHrpsysConfigurator):
    hc = None
    hc_svc = None

    def connectComps(self):
        ChoreonoidHrpsysConfigurator.connectComps(self)
        if self.rh.port("servoState") != None:
            if self.hc:
                connectPorts(self.rh.port("servoState"), self.hc.port("servoStateIn"))

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
            ['te', "ThermoEstimator"],
            # ['tl', "ThermoLimiter"],
            ['rfu', "ReferenceForceUpdater"],
            ['octd', "ObjectContactTurnaroundDetector"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]

    def getRTCInstanceList(self, verbose=True):
        '''!@brief
        Get list of RTC Instance
        '''
        ret = [self.rh, self.hc]
        for rtc in self.getRTCList():
            r = 'self.'+rtc[0]
            try:
                if eval(r): 
                    ret.append(eval(r))
                else:
                    if verbose:
                        print(self.configurator_name + '\033[31mFail to find instance ('+str(rtc)+') for getRTCInstanceList\033[0m')
            except Exception:
                _, e, _ = sys.exc_info()
                print(self.configurator_name + '\033[31mFail to getRTCInstanceList'+str(e)+'\033[0m')
        return ret

    def waitForHRP3HandController(self, robotname="Robot"):
        '''!@brief
        Wait for RobotHardware is exists and activated.

        @param robotname str: name of RobotHardware component.
        '''
        self.hc = None
        timeout_count = 0
        # wait for simulator or RobotHardware setup which sometime takes a long time
        while self.hc == None and timeout_count < 10:  # <- time out limit
            if timeout_count > 0: # do not sleep initial loop
                time.sleep(1);
            self.hc = rtm.findRTC("hc")
            if not self.hc:
                self.hc = rtm.findRTC("HRP3HandController_choreonoid0")
            print(self.configurator_name + "wait for %s : %s ( timeout %d < 10)" % ( "HRP3HandController_choreonoid0", self.hc, timeout_count))
            if self.hc and self.hc.isActive() == None:  # just in case rh is not ready...
                self.hc = None
            timeout_count += 1
        if not self.hc:
            print(self.configurator_name + "Could not find " + "HRP3HandController_choreonoid0")
            if self.ms:
                print(self.configurator_name + "Candidates are .... " + str([x.name()  for x in self.ms.get_components()]))
            print(self.configurator_name + "Exitting.... " + "HRP3HandController_choreonoid0")
            exit(1)

        print(self.configurator_name + "findComps -> %s : %s isActive? = %s " % (self.hc.name(), self.hc,  self.hc.isActive()))

    def waitForRTCManagerAndRobotHardware(self, robotname="Robot", managerhost=nshost):
        '''!@brief
        Wait for both RTC Manager (waitForRTCManager()) and RobotHardware (waitForRobotHardware())

        @param managerhost str: name of host computer that manager is running
        @param robotname str: name of RobotHardware component.
        '''
        self.waitForRTCManager(managerhost)
        self.waitForRobotHardware(robotname)
        self.waitForHRP3HandController()
        self.checkSimulationMode()

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
        self.ic_svc.startImpedanceControllerNoWait("rarm")
        self.ic_svc.startImpedanceControllerNoWait("larm")
        self.ic_svc.waitImpedanceControllerTransition("rarm")
        self.ic_svc.waitImpedanceControllerTransition("larm")
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
