/* Author: Mincheul Kang */

#ifndef OMPL_BASE_HARMONIOUS_SAMPLER_
#define OMPL_BASE_HARMONIOUS_SAMPLER_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/SpaceInformation.h"
#include <string>
#include <map>
#include "ompl/util/RandomNumbers.h"

#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include "boost/multi_array.hpp"
#include <cassert>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>

#include <geometry_msgs/Pose.h>
#include <harmonious_msgs/PossibleConf.h>
#include <harmonious_sampling/HarmoniousParameters.h>
#include <harmonious_sampling/ManipulationRegion.h>

namespace ompl
{
    namespace base
    {
        class HarmoniousSampler{
        public:
//            HarmoniousSampler(){
//
//            }
            ~HarmoniousSampler();

            HarmoniousSampler(std::vector<geometry_msgs::Pose> &grasp_poses,
                              std::vector<harmonious_msgs::BaseConf> &base_confs,
                              std::vector<harmonious_msgs::JointConf> &joint_confs,
                              HarmoniousParameters &params,
                              MainpulationRegion &mr,
                              std::vector<double> &defined_joint_conf);
            HarmoniousParameters                    params_;
            bool makeGoal(const SpaceInformationPtr si_, ompl::base::State *workState);
            bool getReachable(ompl::base::State *workState);
            void jointSampling(ompl::base::State *workState);
            void uniformSampling(ompl::base::State *workState);
            void biasedSampling(ompl::base::State *workState);
            void setBasicPose(ompl::base::State *workState);
            std::vector<int> findIndex(const double value);
            int bSearch(double val, int first, int last);
            int bSearch(double val, int first, int last, int x);
            int bSearch(double val, int first, int last, int x, int y);
            std::vector<int>                             ddd;
        private:
            std::vector<geometry_msgs::Pose>                &grasp_poses_;
            std::vector<harmonious_msgs::BaseConf>          &base_confs_;
            std::vector<harmonious_msgs::JointConf>         &joint_confs_;
            uint                                            size_of_gp;
            std::vector<double>                             defined_joint_conf_;

            // manipulation regions
            double                                          ***prob_dist_;
            double                                          ***cumul_dist_;

            RNG                                             rng_;
            uint                                            num_possible_confs_;
            uint                                            num_dof_;
            MainpulationRegion                              &mr_;
        };
    }
}

#endif
