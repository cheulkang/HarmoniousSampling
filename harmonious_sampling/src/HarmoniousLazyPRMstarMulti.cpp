/* Author: Mincheul Kang */

#include <harmonious_sampling/HarmoniousLazyPRMstarMulti.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/util/GeometricEquations.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/lookup_edge.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include <queue>
#include <stack>
#include <utility>
#include <numeric>
#include <stdio.h>
#include <iostream>

#define foreach BOOST_FOREACH

namespace ompl {
    namespace magic {
/** \brief The number of nearest neighbors to consider by
      default in the construction of the PRM roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS_LAZY = 5;

/** \brief When optimizing solutions with lazy planners, this is the minimum
      number of path segments to add before attempting a new optimized solution
          extraction */
        static const unsigned int MIN_ADDED_SEGMENTS_FOR_LAZY_OPTIMIZATION = 5;
    }
}

ompl::geometric::HarmoniousLazyPRMstarMulti::HarmoniousLazyPRMstarMulti(const base::SpaceInformationPtr &si,
                                                                        base::HarmoniousSampler &hs,
                                                                        const std::vector<bool> &isContinuous,
                                                                        bool starStrategy) :
        base::Planner(si, "HarmoniousLazyPRMstarMulti"),
        starStrategy_(starStrategy),
        userSetConnectionStrategy_(false),
        maxDistance_(0.0),
        indexProperty_(boost::get(boost::vertex_index_t(), g_)),
        stateProperty_(boost::get(vertex_state_t(), g_)),
        radiusProperty_(boost::get(vertex_radius_t(), g_)),
        witnessProperty_(boost::get(vertex_witness_t(), g_)),
        costProperty_(boost::get(vertex_cost_t(), g_)),
        childrenProperty_(boost::get(vertex_children_t(), g_)),
        predecessorProperty_(boost::get(boost::vertex_predecessor_t(), g_)),
        colorProperty_(boost::get(boost::vertex_color_t(), g_)),
        weightProperty_(boost::get(boost::edge_weight_t(), g_)),
        vertexValidityProperty_(boost::get(vertex_flags_t(), g_)),
        edgeValidityProperty_(boost::get(edge_flags_t(), g_)),
        bestCost_(std::numeric_limits<double>::quiet_NaN()),
        iterations_(0),
        increaseIterations_(0),
        BisectionCC_(true),
        rewireFactor_(1.5),
        hs_(hs),
        isContinuous_(isContinuous)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = true;

    Planner::declareParam<bool>("BisectionCC", this,
                                &HarmoniousLazyPRMstarMulti::setBisectionCC, std::string("."));
    Planner::declareParam<double>("RewireFactor", this,
                                  &HarmoniousLazyPRMstarMulti::setRewireFactor, std::string("."));

    addPlannerProgressProperty("iterations INTEGER",
                               std::bind(&HarmoniousLazyPRMstarMulti::getIterationCount, this));
    addPlannerProgressProperty("best cost REAL",
                               std::bind(&HarmoniousLazyPRMstarMulti::getBestCost, this));
    addPlannerProgressProperty("milestone count INTEGER",
                               std::bind(&HarmoniousLazyPRMstarMulti::getMilestoneCountString, this));
    addPlannerProgressProperty("edge count INTEGER",
                               std::bind(&HarmoniousLazyPRMstarMulti::getEdgeCountString, this));
}

ompl::geometric::HarmoniousLazyPRMstarMulti::~HarmoniousLazyPRMstarMulti() {
}

void ompl::geometric::HarmoniousLazyPRMstarMulti::setup() {
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_) {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction(std::bind(&HarmoniousLazyPRMstarMulti::distanceFunctionHarmonious, this, std::placeholders::_1, std::placeholders::_2));
    }

    if (!nnB_) {
        nnB_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nnB_->setDistanceFunction(std::bind(&HarmoniousLazyPRMstarMulti::distanceFunctionHarmonious, this, std::placeholders::_1, std::placeholders::_2));
    }

    if (!nnM_) {
        nnM_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nnM_->setDistanceFunction(std::bind(&HarmoniousLazyPRMstarMulti::distanceFunctionHarmonious, this, std::placeholders::_1, std::placeholders::_2));
    }

    if (!connectionStrategy_) {
//        if (starStrategy_)
//            connectionStrategy_ = KStarStrategy<Vertex>(std::bind(&HarmoniousLazyPRMstarMulti::milestoneCount, this), nn_, si_->getStateDimension());
//        else
//            connectionStrategy_ = KBoundedStrategy<Vertex>(magic::DEFAULT_NEAREST_NEIGHBORS_LAZY, maxDistance_, nn_);
    }

    double dimDbl = static_cast<double>(si_->getStateDimension());
    // double prunedMeasure_ = si_->getSpaceMeasure();

    k_rrgConstant_ = rewireFactor_ * boost::math::constants::e<double>() + (boost::math::constants::e<double>() / dimDbl);

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_) {
        if (pdef_->hasOptimizationObjective()) {
            opt_ = pdef_->getOptimizationObjective();
        }
        else {
            opt_.reset(new base::PathLengthOptimizationObjective(si_));

            if (!starStrategy_) {
                opt_->setCostThreshold(opt_->infiniteCost());
            }
        }
    }
    else {
        OMPL_ERROR("%s: problem definition is not set, deferring setup completion...\n", getName().c_str());
        setup_ = false;
    }
    sampler_ = si_->allocStateSampler();
}

void ompl::geometric::HarmoniousLazyPRMstarMulti::setRange(double distance) {
    maxDistance_ = distance;

    if (!userSetConnectionStrategy_) {
        connectionStrategy_.clear();
    }

    if (isSetup()) {
        setup();
    }
}

void ompl::geometric::HarmoniousLazyPRMstarMulti::setMaxNearestNeighbors(unsigned int k) {
    if (starStrategy_) {
        throw Exception("Cannot set the maximum nearest neighbors for " + getName());
    }

    if (!nn_) {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nn_->setDistanceFunction(std::bind(&HarmoniousLazyPRMstarMulti::distanceFunctionHarmonious, this,std::placeholders::_1, std::placeholders::_2));
    }

    if (!nnB_) {
        nnB_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nnB_->setDistanceFunction(std::bind(&HarmoniousLazyPRMstarMulti::distanceFunctionHarmonious, this, std::placeholders::_1, std::placeholders::_2));
    }

    if (!nnM_) {
        nnM_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
        nnM_->setDistanceFunction(std::bind(&HarmoniousLazyPRMstarMulti::distanceFunctionHarmonious, this, std::placeholders::_1, std::placeholders::_2));
    }

    if (!userSetConnectionStrategy_) {
        connectionStrategy_.clear();
    }

    if (isSetup()) {
        setup();
    }
}

void ompl::geometric::HarmoniousLazyPRMstarMulti::setProblemDefinition(const base::ProblemDefinitionPtr &pdef) {
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::geometric::HarmoniousLazyPRMstarMulti::clearQuery() {
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::geometric::HarmoniousLazyPRMstarMulti::clear() {
    Planner::clear();
    freeMemory();

    if (nn_) {
        nn_->clear();
    }

    if (nnB_) {
        nnB_->clear();
    }

    if (nnM_) {
        nnM_->clear();
    }

    clearQuery();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
}

void ompl::geometric::HarmoniousLazyPRMstarMulti::freeMemory() {
    foreach (Vertex v, boost::vertices(g_)) {
        si_->freeState(stateProperty_[v]);
    }

    g_.clear();
}

// Add newly sampled vertex and its adjancency edges connected to neigh neighbors.
ompl::geometric::HarmoniousLazyPRMstarMulti::Vertex ompl::geometric::HarmoniousLazyPRMstarMulti::addMilestone(base::State *state, bool isM, bool isChecked) {
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    radiusProperty_[m] = std::numeric_limits<double>::infinity();
    costProperty_[m] = std::numeric_limits<double>::infinity();
    childrenProperty_[m] = new std::vector<Vertex>();
    witnessProperty_[m] = NULL;
    predecessorProperty_[m] = NULL;
    colorProperty_[m] = 0;
    vertexValidityProperty_[m] = (isChecked) ? VALIDITY_TRUE : VALIDITY_UNKNOWN;

    std::vector<Vertex> neighbors;
    std::vector<double> neighbors_costs;

    if(isM){
        // manipulation regions
        unsigned long int bSize = nnB_->size();
        unsigned int max_number_of_neighbors = std::ceil(k_rrgConstant_ * log(static_cast<double>(milestoneCount()-bSize) + 1u));
        neighbors.reserve(max_number_of_neighbors);
        neighbors_costs.reserve(max_number_of_neighbors);

        nnM_->nearestK(m, max_number_of_neighbors, neighbors);

        foreach (Vertex n, neighbors) {
            const double weight = distanceFunctionHarmonious(n, m);

            ompl::base::Cost cost_weight(weight);
            const Graph::edge_property_type properties(cost_weight);

            neighborProperty_[m].push_back(type_neighbor(n, weight));
            neighborProperty_[n].push_back(type_neighbor(m, weight));

            // If collision-free or optimized well,
            const Edge &e = boost::add_edge(n, m, properties, g_).first;
            edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
        }

        // base regions
        max_number_of_neighbors = std::ceil(k_rrgConstant_ * log(static_cast<double>(bSize) + 1u));

        neighbors.reserve(max_number_of_neighbors);
        neighbors_costs.reserve(max_number_of_neighbors);

        nnB_->nearestK(m, max_number_of_neighbors, neighbors);

        foreach (Vertex n, neighbors) {
            const double weight = distanceFunctionHarmonious(n, m);

            ompl::base::Cost cost_weight(weight);
            const Graph::edge_property_type properties(cost_weight);

            neighborProperty_[m].push_back(type_neighbor(n, weight));
            neighborProperty_[n].push_back(type_neighbor(m, weight));

            // If collision-free or optimized well,
            const Edge &e = boost::add_edge(n, m, properties, g_).first;
            edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
        }

        nnM_->add(m);
    }
    else{
        unsigned int max_number_of_neighbors = std::ceil(k_rrgConstant_ * log(static_cast<double>(milestoneCount()) + 1u));

        neighbors.reserve(max_number_of_neighbors);
        neighbors_costs.reserve(max_number_of_neighbors);

        nn_->nearestK(m, max_number_of_neighbors, neighbors);

        foreach (Vertex n, neighbors) {
            const double weight = distanceFunctionHarmonious(n, m);

            ompl::base::Cost cost_weight(weight);
            const Graph::edge_property_type properties(cost_weight);

            neighborProperty_[m].push_back(type_neighbor(n, weight));
            neighborProperty_[n].push_back(type_neighbor(m, weight));

            // If collision-free or optimized well,
            const Edge &e = boost::add_edge(n, m, properties, g_).first;
            edgeValidityProperty_[e] = VALIDITY_UNKNOWN;
        }

        nnB_->add(m);
    }

    nn_->add(m);

    return m;
}

ompl::base::PlannerStatus ompl::geometric::HarmoniousLazyPRMstarMulti::solve(const base::PlannerTerminationCondition &ptc) {
    // Initial checkup for start/goal configurations.
    checkValidity();

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart()) {
        Vertex st_vert = addMilestone(si_->cloneState(st), false);
        costProperty_[st_vert] = 0.0; // Initialize with 0 cost.
        startM_.push_back(st_vert);
    }

    if (startM_.size() == 0) {
        OMPL_ERROR("error-%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    bestCost_ = opt_->infiniteCost();
    base::State *workState = si_->allocState();
    bool fullyOptimized = false;
    base::PathPtr bestSolution;

    base::RealVectorStateSpace::StateType *rstate = static_cast<base::RealVectorStateSpace::StateType*>(workState);

    Vertex startV = startM_[0];

    do{
        if(hs_.makeGoal(si_, workState)){
            goalM_.push_back(addMilestone(si_->cloneState(workState), true));
        }
    } while(goalM_.size() == 0 && ptc == false);

    bool isM = false;
    while (ptc == false) {
        ++iterations_;

        int r_val = std::rand() % 100;
        if(r_val < hs_.params_.prob_goal_ && goalM_.size() < hs_.params_.max_num_goal_){
            if(hs_.makeGoal(si_, workState)){
                goalM_.push_back(addMilestone(si_->cloneState(workState), true));
            }
            else{
                continue;
            }
        }
        else if(r_val < hs_.params_.prob_uniform_){
            hs_.uniformSampling(workState);
            if(hs_.getReachable(workState)){
                isM = true;
            }
            else{
                isM = false;
            }
        }
        else{
            hs_.biasedSampling(workState);

            if(hs_.getReachable(workState)){
                hs_.jointSampling(workState);
                isM = true;
            }
            else{
                hs_.setBasicPose(workState);
                isM = false;
            }
        }

        if (!si_->isValid(workState)) {
            continue;
        }

        // Add collision-free vertices.
        Vertex addedVertex = addMilestone(si_->cloneState(workState), isM);

        // DSPT update.
        Decrease(addedVertex);

        // Only support a single pair of start and goal node.
        base::PathPtr solution;

        for (int i = 0; i < (int)goalM_.size(); i++) {
            Vertex goalV = goalM_[i];
            do {
                if (predecessorProperty_[goalV] == NULL || bestCost_.value() <= costProperty_[goalV])
                    break;
                solution = constructSolution(startV, goalV);
            } while (!solution);

            if (solution) {
                base::Cost c(costProperty_[goalV]);

                if (opt_->isCostBetterThan(c, bestCost_)) {
                    bestSolution = solution;

                    bestCost_ = c;
                }
            }
        }
    }
    if (goalM_.empty()) {
        OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    si_->freeState(workState);

    if (bestSolution) {
        base::PlannerSolution psol(bestSolution);
        psol.setPlannerName(getName());
        // If the solution was optimized, we mark it as such
        psol.setOptimized(opt_, bestCost_, fullyOptimized);
        pdef_->addSolutionPath(psol);
    }

    OMPL_INFORM("%s: Created %lu vertices and %lu edges.", getName().c_str(), boost::num_vertices(g_) - 1, boost::num_edges(g_));
    OMPL_INFORM("%s: Goalstates %lu ", getName().c_str(), goalM_.size());
    OMPL_INFORM("Cost: %.6f", bestCost_.value());

    return bestSolution ? base::PlannerStatus::EXACT_SOLUTION :
           base::PlannerStatus::TIMEOUT;
}

// outedge, inedge? - doesn't matter, need to scan all the neighbors.
void ompl::geometric::HarmoniousLazyPRMstarMulti::Decrease(const Vertex &v) {
    typedef std::pair<double, Vertex> weight_vertex;
    std::priority_queue<weight_vertex> pq;

    // Initialize cost of v, i.e., finding best parent vertex in G(g_).
    BGL_FORALL_OUTEDGES(v, e, g_, Graph) {
        Vertex w = target(e, g_);
        double weight = weightProperty_[e].value();

        if (costProperty_[v] > costProperty_[w] + weight) {
            predecessorProperty_[v] = w;
            costProperty_[v] = costProperty_[w] + weight;
        }
    }

    // No need to invoke cancelAdoption since v is newly sampled.
    if (predecessorProperty_[v] != NULL) {
        childrenProperty_[predecessorProperty_[v]]->push_back(v);
    }

    // At this point, v has a best parent. From now on construct its subtree of descendants.

    pq.push(weight_vertex(-costProperty_[v], v)); // Invert the cost value for mimicking min-heap.

    while (!pq.empty()) {
        weight_vertex top = pq.top();
        pq.pop();

        double cost = -top.first; // Invert the cost value to be like min-heap.
        Vertex vert = top.second;

        if (cost > costProperty_[vert]) {
            continue;
        }

        BGL_FORALL_OUTEDGES(vert, e, g_, Graph) {
            Vertex w = target(e, g_);
            double weight = weightProperty_[e].value();
            double cost_w = costProperty_[w];

            if (cost_w > cost + weight) {
                costProperty_[w] = cost + weight;
                cancelAdoption(w);

                predecessorProperty_[w] = vert;
                childrenProperty_[vert]->push_back(w);
                pq.push(weight_vertex(-costProperty_[w], w));
            }
        }
    }

    // Now, DSPT is stable.
}

#define RED (increaseIterations_ + 1) // I know, it's bad #define.

void ompl::geometric::HarmoniousLazyPRMstarMulti::Increase(const Vertex vs) {
    // <Step 1. Preparation.
    // white is used for color of each vertex without initialization.
    // For each iteration, white is increased by 1, thus we can use it as
    // equal to or less than 'white' means 'white' color, 'red' otherwise.
    increaseIterations_ += 1;

    std::vector<Vertex> reds;
    typedef std::pair<double, Vertex> weight_vertex;
    std::priority_queue<weight_vertex> pq; //  Max-heap by default.

    pq.push(weight_vertex(-costProperty_[vs], vs));  // It works as if it is min-heap.

    // <Step 2. Coloring
    while (!pq.empty()) {
        weight_vertex top = pq.top();
        pq.pop();

        double cost = -top.first;
        Vertex vert = top.second;

        if (cost > costProperty_[vert]) {
            continue;  // Instead of heap-improve
        }

        // Probability to get a pink node? almost impossible!
        /*
        // If there exist a non-red neighbor q of z such that Dist(q) + w_(q, z) = D(z)
        // set pink, that means it can keep the current cost, thus it is not necessary to
        // iterate its children.
        // Otherwise, set red and enqueue all the children of z.
        bool pink_flag = false;
        BGL_FORALL_OUTEDGES(vert, e, g_, Graph) {
          Vertex w = target(e, g_);
          double weight = weightProperty_[e].value();

          if (colorProperty_[w] != RED && costProperty_[w] + weight == cost) {
            // Actually, '<' should not be happened all the time.
            // And even '==' would very rarely occur, but possible.
            pink_flag = true;

            cancelAdoption(vert);
            predecessorProperty_[vert] = w;
            childrenProperty_[w]->push_back(vert);
            break; // If there exsits, take anyone among them.
          }
        }

        if (pink_flag) {
          continue;
        }*/

        colorProperty_[vert] = RED; // Set to 'red'
        reds.push_back(vert);
        // Even with multiple starting red nodes, there will be no re-visit since each starting node is
        // a root node of sub'tree' in DSPT. That is, if statement within for loop might be useless.
        // Someone would be curious, e.g., then why do we have to use priority queue in step2 ?
        // Just for 'pink' case. Yeap. We need to identify all the other parent candidates are red or not
        // prior to checking current node.
        std::vector<Vertex> *children = childrenProperty_[vert];

        for (unsigned int i = 0; i < children->size(); i++) {
            pq.push(weight_vertex(-costProperty_[(*children)[i]], (*children)[i]));
        }
    }

    // 'pq' is empty at here.

    // <Step 3-a. Find best non-red parent for each red node.
    for (unsigned int i = 0; i < reds.size(); i++) {
        // TODO : need to be verified
        // Cost/predecessor initialization.
        costProperty_[reds[i]] = std::numeric_limits<double>::infinity();
        cancelAdoption(reds[i]);

        BGL_FORALL_OUTEDGES(reds[i], e, g_, Graph) {
            Vertex w = target(e, g_);
            double weight = weightProperty_[e].value();

            if (colorProperty_[w] == RED) {
                continue;  // If red, put aside for a while.
            }

            if (costProperty_[reds[i]] > costProperty_[w] + weight) {
                costProperty_[reds[i]] = costProperty_[w] + weight;
                predecessorProperty_[reds[i]] = w;
            }
        }

        if (predecessorProperty_[reds[i]] != NULL) {
            childrenProperty_[predecessorProperty_[reds[i]]]->push_back(reds[i]);
        }

        pq.push(weight_vertex(-costProperty_[reds[i]], reds[i]));
    }

    // <Step 3-b. Propagate the changes; rewiring for 'red' nodes whether it can replace
    // existing parent node of near neighbors.
    while (!pq.empty()) {
        weight_vertex top = pq.top();
        pq.pop();

        double cost = -top.first;
        Vertex vert = top.second;

        if (costProperty_[vert] < cost) {
            continue;
        }

        BGL_FORALL_OUTEDGES(vert, e, g_, Graph) {
            Vertex w = target(e, g_);
            double weight = weightProperty_[e].value();

            if (colorProperty_[w] != RED) {
                continue;  // If not red, then skip.
            }

            if (cost + weight < costProperty_[w]) {
                costProperty_[w] = cost + weight;

                cancelAdoption(w);

                predecessorProperty_[w] = vert;
                childrenProperty_[vert]->push_back(w);
                pq.push(weight_vertex(-costProperty_[w], w));
            }
        }
    }

    // The end!
    // colorProperty_ is not necessary to be cleansed out, just increase variable, 'RED'.
}

// TODO : sync between children & edges.
void ompl::geometric::HarmoniousLazyPRMstarMulti::cancelAdoption(const Vertex &child) {
    if (predecessorProperty_[child] == NULL)
        return;

    std::vector<Vertex> *children = childrenProperty_[predecessorProperty_[child]];

    for (unsigned int i = 0; i < children->size(); i++) if ((*children)[i] == child) {
            std::swap((*children)[i], children->back());
            children->pop_back();
            break;
        }

    predecessorProperty_[child] = NULL;
}

// Vertex first.
ompl::base::PathPtr ompl::geometric::HarmoniousLazyPRMstarMulti::constructSolution(const Vertex &start, const Vertex &goal) {
    std::vector<Vertex> solution_path;

    // Construct a solution from DSPT.
    for (Vertex vert = goal; vert != NULL; vert = predecessorProperty_[vert])
        solution_path.push_back(vert);

    if (solution_path.empty() || solution_path.size() == 1)
        return base::PathPtr();

    // Goal = 0, Start = n - 1.
    // TODO : From goal or start ? which one is better?

    // auto from = solution_path.rbegin();
    std::vector<Vertex>::reverse_iterator from = solution_path.rbegin();
    for (std::vector<Vertex>::reverse_iterator to = from + 1; to != solution_path.rend(); ++to) {
        Edge e = boost::lookup_edge(*from, *to, g_).first; // Exhaustive search O(E) at worst case.
        unsigned int &evd = edgeValidityProperty_[e];

        if ((evd & VALIDITY_TRUE) == 0) { // Unknown
            bool result = true;
            // double weight = weightProperty_[e].value();

            result &= checkMotion(stateProperty_[*from], stateProperty_[*to]);

            if (result) {
                evd |= VALIDITY_TRUE;
            }
            else {
                boost::remove_edge(e, g_); // O(log(E/V)) time...

                cancelAdoption(*to);
                Increase(*to);
                return base::PathPtr();
            }
        }

        from = to;
    }

    PathGeometric *p = new PathGeometric(si_);

    // Feasible path is found, fetch optimized edges if possible.

    for (std::vector<Vertex>::const_reverse_iterator sol = solution_path.rbegin(); sol != solution_path.rend(); ++sol) {
        p->append(stateProperty_[*sol]);
    }

    return base::PathPtr(p);
}

void ompl::geometric::HarmoniousLazyPRMstarMulti::getPlannerData(base::PlannerData &data) const {
    Planner::getPlannerData(data);
    // Caution : it handles directional information regardless of the search graph setting
    //           which is undirectional graph.
}

double ompl::geometric::HarmoniousLazyPRMstarMulti::distanceFunction(const base::State *a, const base::State *b) const {
    return si_->distance(a, b);
}

double ompl::geometric::HarmoniousLazyPRMstarMulti::distanceFunctionHarmonious(const Vertex a, const Vertex b) const {
    double base = distanceFunctionBase(stateProperty_[a], stateProperty_[b]);
    double arm =  distanceFunctionJoints(stateProperty_[a], stateProperty_[b]);

    return base + arm;
}


double ompl::geometric::HarmoniousLazyPRMstarMulti::distanceFunctionBase(const base::State *a, const base::State *b) const {
    double dist = 0.0;
    int dim = si_->getStateDimension();

    std::vector<double> ca, cb;
    si_->getStateSpace()->copyToReals(ca, a);
    si_->getStateSpace()->copyToReals(cb, b);

    for (int i = 0; i < NUM_BASE_DOF; i++) {
        double dd = fabs(ca[i]-cb[i]);

        if(isContinuous_[i]) {
            if (dd > boost::math::constants::pi<double>()) {
                dd = 2.0 * boost::math::constants::pi<double>() - dd;
            }
        }

        dist += hs_.params_.weights_[i] * dd * hs_.params_.weights_[i] * dd;
    }

    return std::sqrt(dist);
}

double ompl::geometric::HarmoniousLazyPRMstarMulti::distanceFunctionJoints(const base::State *a, const base::State *b) const {
    double dist = 0.0;

    std::vector<double> ca, cb;
    si_->getStateSpace()->copyToReals(ca, a);
    si_->getStateSpace()->copyToReals(cb, b);

    for (int i = NUM_BASE_DOF; i < ca.size(); i++) {
        double dd = fabs(ca[i]-cb[i]);

        if(isContinuous_[i+3]){
            if (dd > boost::math::constants::pi<double>()) {
                dd = 2.0 * boost::math::constants::pi<double>() - dd;
            }
        }

        dist += hs_.params_.weights_[i] * dd * hs_.params_.weights_[i] * dd;
    }

    return std::sqrt(dist);
}


bool ompl::geometric::HarmoniousLazyPRMstarMulti::checkMotion(base::State *s1, base::State *s2) const {
//    /* Assume motion starts/ends in a valid configuration so v1/v2 are valid */
    bool result = true;
    int dim = si_->getStateDimension();
    int nd;

    int nd_j = validSegmentCount_compare(distanceFunctionJoints(s1, s2), hs_.params_.factor_joint_);
    int nd_b = validSegmentCount_compare(distanceFunctionBase(s1, s2), hs_.params_.factor_base_);
    if(nd_j > nd_b){
        nd = nd_j;
    }
    else{
        nd = nd_b;
    }

    if (nd > 1) {
        /* Temporary storage for the checked state */
        base::State *test = si_->allocState();
        std::queue<std::pair<unsigned int, unsigned int> > q;
        q.push(std::make_pair(1, nd - 1));

        while (!q.empty()) {
            std::pair<unsigned int, unsigned int> range = q.front();
            unsigned int mid;

            mid = (range.first + range.second) / 2;
            interpolate(s1, s2, (double)mid / (double)nd, test);
//            si_->getStateSpace()->interpolate(s1, s2, (double)mid / (double)nd, test);

            if (!si_->isValid(test)) {
                result = false;
                break;
            }

            q.pop();
            if (range.first < mid)
                q.push(std::make_pair(range.first, mid - 1));
            if (mid < range.second) {
                q.push(std::make_pair(mid + 1, range.second));
            } // if mid == first, no more recursion.
        }
    }

    return result;
}

unsigned int ompl::geometric::HarmoniousLazyPRMstarMulti::validSegmentCount_compare(const double dist,
                                                                                    const double longestValidSegment) const {
    return si_->getStateSpace()->getValidSegmentCountFactor() * (unsigned int)ceil(dist / longestValidSegment);
}

void ompl::geometric::HarmoniousLazyPRMstarMulti::interpolate(const base::State *from, const base::State *to, const double t, base::State *state) const
{
    const base::RealVectorStateSpace::StateType *rfrom = static_cast<const base::RealVectorStateSpace::StateType*>(from);
    const base::RealVectorStateSpace::StateType *rto = static_cast<const base::RealVectorStateSpace::StateType*>(to);
    const base::RealVectorStateSpace::StateType *rstate = static_cast<base::RealVectorStateSpace::StateType*>(state);
    for (unsigned int i = 0 ; i < isContinuous_.size() ; ++i){
        if(isContinuous_[i]){
            double diff = rto->values[i] - rfrom->values[i];
            if (fabs(diff) <= boost::math::constants::pi<double>())
                rstate->values[i] = rfrom->values[i] + diff * t;
            else
            {
                double &v = rstate->values[i];
                if (diff > 0.0)
                    diff = 2.0 * boost::math::constants::pi<double>() - diff;
                else
                    diff = -2.0 * boost::math::constants::pi<double>() - diff;
                v = rfrom->values[i] - diff * t;
                // input states are within bounds, so the following check is sufficient
                if (v > boost::math::constants::pi<double>())
                    v -= 2.0 * boost::math::constants::pi<double>();
                else
                if (v < -boost::math::constants::pi<double>())
                    v += 2.0 * boost::math::constants::pi<double>();
            }
        }
        else{
            rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;
        }
    }
}
