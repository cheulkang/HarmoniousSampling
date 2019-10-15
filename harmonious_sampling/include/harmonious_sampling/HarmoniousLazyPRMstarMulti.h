/* Author: Mincheul Kang */

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_HARMONIOUS_LAZY_PRM_STAR_MULTI_
#define OMPL_GEOMETRIC_PLANNERS_PRM_HARMONIOUS_LAZY_PRM_STAR_MULTI_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/functional/hash.hpp>
#include <boost/function.hpp>
#include <boost/tuple/tuple.hpp>

#include <utility>
#include <vector>
#include <map>
#include <queue>

#include <harmonious_sampling/HarmoniousSampler.h>

namespace ompl {
    namespace base {
// Forward declare for use in implementation
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }

    namespace geometric {

/**
   @anchor gHarmoniousLazyPRMstarMulti
   @par Short description
   HarmoniousLazyPRMstarMulti is a planner that uses lazy collision checking
   with dynamc shortest path tree.
   @par External documentation
*/

/** \brief Lazy Probabilistic RoadMap planner */
        class HarmoniousLazyPRMstarMulti : public base::Planner {
        public:
            struct vertex_state_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_flags_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_radius_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_witness_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_cost_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_children_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_neighbors_t {
                typedef boost::vertex_property_tag kind;
            };

            struct edge_flags_t {
                typedef boost::edge_property_tag kind;
            };

            /** @brief The type for a vertex in the roadmap. */
            typedef boost::adjacency_list_traits<boost::vecS, boost::listS,
                    boost::undirectedS>::vertex_descriptor Vertex;

            /**
             @brief The underlying roadmap graph.

             @par Any BGL graph representation could be used here. Because we
             expect the roadmap to be sparse (m<n^2), an adjacency_list is more
             appropriate than an adjacency_matrix. We use listS for the vertex list
             because vertex descriptors are invalidated by remove operations if using vecS.

             @par Obviously, a ompl::base::State* vertex property is required.
             The incremental connected components algorithm requires
             vertex_predecessor_t and vertex_rank_t properties.
             If boost::vecS is not used for vertex storage, then there must also
             be a boost:vertex_index_t property manually added.

             @par Edges should be undirected and have a weight property.
             */

            typedef std::pair<Vertex, double> type_neighbor;

            typedef boost::adjacency_list <
                    boost::vecS, boost::listS, boost::undirectedS,
                    // Vertex properties.
                    boost::property < vertex_state_t, base::State *,
                            boost::property < boost::vertex_index_t, unsigned long int,
                                    boost::property < vertex_flags_t, unsigned int,
                                            boost::property < vertex_radius_t, double,
                                                    boost::property < vertex_witness_t, base::State *,
                                                            boost::property < vertex_cost_t, double,
                                                                    boost::property < vertex_children_t, std::vector<Vertex> *,
                                                                            boost::property < vertex_neighbors_t, std::vector<type_neighbor>,
                                                                                    boost::property < boost::vertex_color_t, unsigned int,
                                                                                            boost::property < boost::vertex_predecessor_t, Vertex,
                                                                                                    boost::property < boost::vertex_rank_t, unsigned long int > > > > > > > > > > >,
                    // Edge properties.
                    boost::property < boost::edge_weight_t, base::Cost,
                            boost::property < edge_flags_t, unsigned int > >
            > Graph;

            /** @brief The type for an edge in the roadmap. */
            typedef boost::graph_traits<Graph>::edge_descriptor   Edge;

            /** @brief A nearest neighbors data structure for roadmap vertices. */
            typedef boost::shared_ptr< NearestNeighbors<Vertex> > RoadmapNeighbors;

            /** @brief A function returning the milestones that should be
             * attempted to connect to. */
            typedef boost::function<const std::vector<Vertex>&(const Vertex)> ConnectionStrategy;

            /** \brief Constructor */
            HarmoniousLazyPRMstarMulti(const base::SpaceInformationPtr &si,
                                       base::HarmoniousSampler &hs,
                                       bool starStrategy = false);

            virtual ~HarmoniousLazyPRMstarMulti();

            /** \brief Set the maximum length of a motion to be added to the roadmap. */
            void setRange(double distance);

            /** \brief Get the range the planner is using */
            double getRange() const {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors() {
                nn_.reset(new NN<Vertex>());
                nnB_.reset(new NN<Vertex>());
                nnM_.reset(new NN<Vertex>());

                if (!userSetConnectionStrategy_) {
                    connectionStrategy_.clear();
                }

                if (isSetup()) {
                    setup();
                }
            }

            template <typename Container>
            struct container_hash {
                std::size_t operator()(Container const& c) const {
                    return boost::hash_range(c.begin(), c.end());
                }
            };

            virtual void setProblemDefinition(const base::ProblemDefinitionPtr &pdef);

            /** \brief Set the connection strategy function that specifies the
             milestones that connection attempts will be make to for a
             given milestone.

             \par The behavior and performance of PRM can be changed drastically
             by varying the number and properties if the milestones that are
             connected to each other.

             \param pdef A function that takes a milestone as an argument and
             returns a collection of other milestones to which a connection
             attempt must be made. The default connection strategy is to connect
             a milestone's 10 closest neighbors.
             */
            void setConnectionStrategy(const ConnectionStrategy &connectionStrategy) {
                connectionStrategy_ = connectionStrategy;
                userSetConnectionStrategy_ = true;
            }

            /** \brief Convenience function that sets the connection strategy to the
             default one with k nearest neighbors.
             */
            void setMaxNearestNeighbors(unsigned int k);

            /** \brief Return the number of milestones currently in the graph */
            unsigned long int milestoneCount() const {
                return boost::num_vertices(g_);
            }

            /** \brief Return the number of edges currently in the graph */
            unsigned long int edgeCount() const {
                return boost::num_edges(g_);
            }

            virtual void getPlannerData(base::PlannerData &data) const;
            virtual void setup();
            virtual void clear();

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously computed roadmap,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for HarmoniousLazyPRMstarMulti. */
            void clearQuery();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            void Decrease(const Vertex &v);
            void Increase(const Vertex vs);
            void cancelAdoption(const Vertex &v);

        protected:
            /** \brief Flag indicating validity of an edge of a vertex */
            static const unsigned int VALIDITY_UNKNOWN = 0;

            /** \brief Flag indicating validity of an edge of a vertex */
            static const unsigned int VALIDITY_TRUE    = 1;

            ///////////////////////////////////////
            // Planner progress property functions
            std::string getIterationCount() const {
                return boost::lexical_cast<std::string>(iterations_);
            }
            std::string getBestCost() const {
                return boost::lexical_cast<std::string>(bestCost_);
            }
            std::string getMilestoneCountString() const {
                return boost::lexical_cast<std::string>(milestoneCount());
            }
            std::string getEdgeCountString() const {
                return boost::lexical_cast<std::string>(edgeCount());
            }

            /** \brief Free all the memory allocated by the planner */
            void freeMemory();

            /** \brief Construct a milestone for a given state (\e state), store it in the nearest neighbors data structure
                and then connect it to the roadmap in accordance to the connection strategy. */
            Vertex addMilestone(base::State *state, bool isM, bool isChecked = true);

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as the solution */
            ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

            /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
            double distanceFunction(const base::State *a, const base::State *b) const;
            double distanceFunctionHarmonious(const Vertex a, const Vertex b) const;
            double distanceFunctionBase(const base::State *a, const base::State *b) const;
            double distanceFunctionJoints(const base::State *a, const base::State *b) const;

            bool checkMotion(base::State *s1, base::State *s2) const;
            unsigned int validSegmentCount_compare(const double dist, const double longestValidSegment) const;

            boost::tuple<double, double, double> toEulerAngle(double w, double x, double y, double z) const;
            boost::tuple<double, double, double, double> toQuaternion(double pitch, double roll, double yaw) const;

            /** \brief Given two vertices, returns a heuristic on the cost of the path connecting them.
                This method wraps OptimizationObjective::motionCostHeuristic */
            base::Cost costHeuristic(Vertex u, Vertex v) const;

            // <Dancing compoenents
            // Let's dance!
            std::vector<base::State*>* optimizeMotion(const Vertex &v1, const Vertex &v2);
            // >

            /** \brief Flag indicating whether the default connection strategy is the Star strategy */
            bool                                                   starStrategy_;

            /** \brief Function that returns the milestones to attempt connections with */
            ConnectionStrategy                                     connectionStrategy_;

            /** \brief Flag indicating whether the employed connection strategy was set by the user (or defaults are assumed) */
            bool
                    userSetConnectionStrategy_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                                 maxDistance_;

            /** \brief Sampler user for generating random in the state space */
            base::StateSamplerPtr                                  sampler_;

            /** \brief Nearest neighbors data structure */
            RoadmapNeighbors                                       nn_;
            RoadmapNeighbors                                       nnB_;
            RoadmapNeighbors                                       nnM_;

            /** \brief Connectivity graph */
            Graph                                                  g_;

            /** \brief Array of start milestones */
            std::vector<Vertex>                                    startM_;

            /** \brief Array of goal milestones */
            std::vector<Vertex>                                    goalM_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, boost::vertex_index_t>::type indexProperty_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, vertex_state_t>::type       stateProperty_;

            /** \brief Access to the approximate collision-free radius at each Vertex */
            boost::property_map<Graph, vertex_radius_t>::type radiusProperty_;

            /** \brief Access to the approximate closest obstacle space at each Vertex */
            boost::property_map<Graph, vertex_witness_t>::type witnessProperty_;

            /** \brief Access to the cost from start configuration at each Vertex */
            boost::property_map<Graph, vertex_cost_t>::type costProperty_;

            /** \brief Access to the children of each Vertex */
            boost::property_map<Graph, vertex_children_t>::type childrenProperty_;

            /** \brief Access to the neighbors within a ball of radius r_rrg* of each Vertex */
            boost::property_map<Graph, vertex_neighbors_t>::type neighborProperty_;

            /** \brief Access to the predecessor of each Vertex */
            boost::property_map<Graph, boost::vertex_predecessor_t>::type predecessorProperty_;

            /** \brief Access to the color of each Vertex used in Increase() */
            boost::property_map<Graph, boost::vertex_color_t>::type colorProperty_;

            /** \brief Access to the weights of each Edge */
            boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

            /** \brief Access the validity state of a vertex */
            boost::property_map<Graph, vertex_flags_t>::type       vertexValidityProperty_;

            /** \brief Access the validity state of an edge */
            boost::property_map<Graph, edge_flags_t>::type         edgeValidityProperty_;

            /** \brief Objective cost function for PRM graph edges */
            base::OptimizationObjectivePtr                         opt_;

            base::Cost                                             bestCost_;

            unsigned long int                                      iterations_;

            /** \brief The number of 'Increase' invoked to avoid color initialization. */
            unsigned long int                                      increaseIterations_;

            double                                                 k_rrgConstant_;

            /** \brief A set of parameters configuratable externally. */
            // Toggler
            bool                                                   BisectionCC_;
            void setBisectionCC(bool v) { BisectionCC_ = v; }
            // Parameter
            double rewireFactor_;
            void setRewireFactor(double v) { rewireFactor_ = v; }

            base::HarmoniousSampler                                &hs_;
        };

    }
}

#endif
