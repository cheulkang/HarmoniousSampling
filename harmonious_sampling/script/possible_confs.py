import openravepy
import rospy

import conf_generator
from harmonious_msgs.msg import *
from harmonious_msgs.srv import *

def handle_conf_generator(req):
    # request
    gp = req.grasp_poses[0]
    m_gp = openravepy.matrixFromPose((gp.orientation.w, gp.orientation.x, gp.orientation.y, gp.orientation.z,\
                               gp.position.x, gp.position.y, gp.position.z))

    bps = cg._find_base_conf(m_gp, req.base_num)

    # response
    pcr = PossibleConfResponse()
    i = 0
    if bps is not None:
        for b in bps[0]:
            axisAngle = openravepy.axisAngleFromQuat(b[0:4])
            bm = [b[4], b[5], axisAngle[2]]
            bc = BaseConf()
            bc.b_c.append(bm[0])
            bc.b_c.append(bm[1])
            bc.b_c.append(bm[2])
            pcr.base_confs.append(bc)

            jc = JointConf()
            if(i < req.joint_num):
                robot.SetActiveDOFValues(bm)
                joint_conf = manip.FindIKSolution(m_gp, filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions)
                if joint_conf is not None:
                    jc.j_c = joint_conf
                    i = i + 1
            pcr.joint_confs.append(jc)
    return pcr

if __name__ == "__main__":
    rospy.init_node('conf_generator', anonymous=True)

    env = openravepy.Environment()
    # env.SetViewer('qtcoin')
    env.Load('./robot.xml')
    robot = env.GetRobots()[0]
    arm = 'rightarm'

    manip = robot.SetActiveManipulator(arm)
    robot.SetActiveDOFs([], openravepy.DOFAffine.X | openravepy.DOFAffine.Y | openravepy.DOFAffine.RotationAxis, [0, 0, 1])

    cg = conf_generator.ConfGenerator(robot, arm)

    # ros service
    s = rospy.Service('possible_conf_generator', PossibleConf, handle_conf_generator)
    rospy.spin()
