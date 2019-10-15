/* Author: Mincheul Kang */

#ifndef HARMONIOUS_SAMPLING_HARMONIOUSPARAMETERS_H
#define HARMONIOUS_SAMPLING_HARMONIOUSPARAMETERS_H

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/goals/GoalStates.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#define NUM_BASE_DOF 3

class HarmoniousParameters
{
public:
    HarmoniousParameters(ompl::base::RealVectorBounds &bounds);
    ~HarmoniousParameters();

    double                                  resolution_;
    double                                  res_yaw_;
    std::vector<double>                     weights_;
    ompl::base::RealVectorBounds            bounds_;
    uint                                    reg_index_[NUM_BASE_DOF];
    double                                  prob_goal_;
    double                                  prob_uniform_;
    double                                  max_num_goal_;
    double                                  interval_[NUM_BASE_DOF];

    double                                  factor_base_;
    double                                  factor_joint_;

    double                                  v_base_;
    double                                  v_manip_;
};

#endif //HARMONIOUS_SAMPLING_HARMONIOUSPARAMETERS_H
