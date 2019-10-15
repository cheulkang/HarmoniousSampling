import openravepy
import numpy as np

class ConfGenerator():
    def __init__(self, robot, arm='rightarm'):
        self.robot = robot

        # set the manipulator
        self.robot.SetActiveManipulator(arm)

        ### inverse kinematics
        self.ikmodel = self._load_ik_model()
        ### inverse reachability
        self.irmodel = self._load_ir_model()

    def _load_ik_model(self):
        ik = openravepy.databases.inversekinematics.InverseKinematicsModel(robot=self.robot, iktype=openravepy.IkParameterization.Type.Transform6D)
        if not ik.load():
            print("Construct a inverse kinematics database")
            ik.autogenerate()
            return self._load_ik_model()
        else:
            return ik

    def _load_ir_model(self):
        ir = openravepy.databases.inversereachability.InverseReachabilityModel(robot=self.robot)
        if not ir.load():
            print("Construct a inverse reachability database")
            ir.autogenerate()
            return self._load_ir_model()
        else:
            return ir

    def _find_base_conf(self, Tgrasp, numOfSample=600):
        densityfn, samplerfn, bounds = self.irmodel.computeBaseDistribution(Tgrasp)
        """
        densityfn: gaussian kernel density function taking poses of openrave quaternion type, returns probabilities
        samplerfn: gaussian kernel sampler function taking number of sample and weight, returns robot base poses and joint states
        bounds: 2x3 array, bounds of samples, [[min rotation, min x, min y],[max rotation, max x, max y]]
        """
        if bounds is None:
            return None

        confs, jointState = samplerfn(numOfSample)
        return confs, densityfn(confs)