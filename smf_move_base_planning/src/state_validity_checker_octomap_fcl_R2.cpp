/*! \file state_validity_checker_octomap_fcl_R2.cpp
 * \brief State validity checker.
 *
 * \date March 5, 2015
 * \author Juan David Hernandez Vega, juandhv@rice.edu
 *
 * \details Check is a given configuration R2 is collision-free.
 *  The workspace is represented by an Octomap and collision check is done with FCL.
 *
 * Based on Juan D. Hernandez Vega's PhD thesis, University of Girona
 * http://hdl.handle.net/10803/457592, http://www.tdx.cat/handle/10803/457592
 */

#include <state_validity_checker_octomap_fcl_R2.h>

OmFclStateValidityCheckerR2::OmFclStateValidityCheckerR2(const ob::SpaceInformationPtr &si,
                                                         const bool opport_collision_check,
                                                         std::vector<double> planning_bounds_x,
                                                         std::vector<double> planning_bounds_y)
    : ob::StateValidityChecker(si), local_nh_("~"), robot_base_radius_(0.4), robot_base_height_(2.0)
{
    GetOctomap::Request req;
    GetOctomap::Response resp;

    opport_collision_check_ = opport_collision_check;
    planning_bounds_x_ = planning_bounds_x;
    planning_bounds_y_ = planning_bounds_y;

    local_nh_.param("robot_base_radius", robot_base_radius_, robot_base_radius_);
    local_nh_.param("robot_base_height", robot_base_height_, robot_base_height_);
    local_nh_.param("octomap_service", octomap_service_, octomap_service_);
    local_nh_.param("odometry_topic", odometry_topic, odometry_topic);
    local_nh_.param("main_frame", main_frame, main_frame);
    local_nh_.param("optimization_objective", optimization_objective, optimization_objective);
    local_nh_.param("social_costmap_topic", social_costmap_topic, social_costmap_topic);

    octree_ = NULL;

    ROS_DEBUG("%s: requesting the map to %s...", ros::this_node::getName().c_str(),
              nh_.resolveName(octomap_service_).c_str());

    while ((nh_.ok() && !ros::service::call(octomap_service_, req, resp)) || resp.map.data.size() == 0)
    {
        ROS_WARN("Request to %s failed; trying again...", nh_.resolveName(octomap_service_).c_str());
        usleep(1000000);
    }
    if (nh_.ok())
    { // skip when CTRL-C
        abs_octree_ = octomap_msgs::msgToMap(resp.map);
        std::cout << std::endl;
        if (abs_octree_)
        {
            octree_ = dynamic_cast<octomap::OcTree *>(abs_octree_);
            tree_ = new fcl::OcTreef(std::shared_ptr<const octomap::OcTree>(octree_));
            tree_obj_ = new fcl::CollisionObjectf((std::shared_ptr<fcl::CollisionGeometryf>(tree_)));
        }

        robot_collision_solid_.reset(new fcl::Cylinderf(robot_base_radius_, robot_base_height_));

        agent_collision_solid_.reset(new fcl::Cylinderf(0.35, robot_base_height_));

        octree_res_ = octree_->getResolution();
        octree_->getMetricMin(octree_min_x_, octree_min_y_, octree_min_z_);
        octree_->getMetricMax(octree_max_x_, octree_max_y_, octree_max_z_);

        if (octree_)
            ROS_DEBUG("%s: Octomap received (%zu nodes, %f m res)", ros::this_node::getName().c_str(),
                      octree_->size(), octree_->getResolution());
        else
            ROS_ERROR("Error reading OcTree from stream");
    }

    ROS_INFO_STREAM("Retrieving social costmap.");
    socialCostmap = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(social_costmap_topic);
    socialCostmapValues = socialCostmap->data;
    // ROS_INFO_STREAM("social costmap values: " << this->socialCostmap->data);
}

bool OmFclStateValidityCheckerR2::isValid(const ob::State *state) const
{
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    // ompl::tools::Profiler::Begin("collision");

    // extract the component of the state and cast it to what we expect

    if (opport_collision_check_ &&
        (state_r2->values[0] < octree_min_x_ || state_r2->values[1] < octree_min_y_ ||
         state_r2->values[0] > octree_max_x_ || state_r2->values[1] > octree_max_y_))
    {
        // ompl::tools::Profiler::End("collision");
        return true;
    }

    if (state_r2->values[0] < planning_bounds_x_[0] || state_r2->values[1] < planning_bounds_y_[0] ||
        state_r2->values[0] > planning_bounds_x_[1] || state_r2->values[1] > planning_bounds_y_[1])
    {
        // ompl::tools::Profiler::End("collision");
        return false;
    }

    // FCL
    fcl::Transform3f robot_tf;
    robot_tf.setIdentity();
    robot_tf.translate(fcl::Vector3f(state_r2->values[0], state_r2->values[1], robot_base_height_ / 2.0));

    fcl::CollisionObjectf vehicle_co(robot_collision_solid_, robot_tf);

    fcl::CollisionRequestf collision_request;
    fcl::CollisionResultf collision_result;

    fcl::collide(tree_obj_, &vehicle_co, collision_request, collision_result);

    // std::cout << "Collision (FCL): " << collision_result.isCollision() << std::endl;

    if (collision_result.isCollision())
    {
        // ompl::tools::Profiler::End("collision");
        return false;
    }

    return true;
}

double OmFclStateValidityCheckerR2::clearance(const ob::State *state) const
{
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();
    double minDist = std::numeric_limits<double>::infinity();

    // ompl::tools::Profiler::Begin("clearance");

    // extract the component of the state and cast it to what we expect

    if (opport_collision_check_ &&
        (state_r2->values[0] < octree_min_x_ || state_r2->values[1] < octree_min_y_ ||
         state_r2->values[0] > octree_max_x_ || state_r2->values[1] > octree_max_y_))
    {
        // ompl::tools::Profiler::End("clearance");
        return minDist;
    }

    // FCL
    fcl::Transform3f vehicle_tf;
    vehicle_tf.setIdentity();
    vehicle_tf.translate(fcl::Vector3f(state_r2->values[0] + 3.5, state_r2->values[1], 0.0));
    // fcl::Quaternion3f qt0;
    // qt0.fromEuler(0.0, 0.0, 0.0);
    // vehicle_tf.setQuatRotation(qt0);

    fcl::CollisionObjectf vehicle_co(robot_collision_solid_, vehicle_tf);
    fcl::DistanceRequestf distanceRequest;
    fcl::DistanceResultf distanceResult;

    fcl::distance(tree_obj_, &vehicle_co, distanceRequest, distanceResult);

    // std::cout << "Distance (FCL): " << distanceResult.min_distance << std::endl;

    if (distanceResult.min_distance < minDist)
        minDist = distanceResult.min_distance;

    // ompl::tools::Profiler::End("clearance");

    return minDist;
}

double OmFclStateValidityCheckerR2::checkRiskZones(const ob::State *state) const
{
    ROS_INFO_STREAM("running risk zone function");
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();
    double state_risk = 1.0;

    // ompl::tools::Profiler::Begin("RiskZones");

    // extract the component of the state and cast it to what we expect

    if (opport_collision_check_ &&
        (state_r2->values[0] < octree_min_x_ || state_r2->values[1] < octree_min_y_ ||
         state_r2->values[0] > octree_max_x_ || state_r2->values[1] > octree_max_y_))
    {
        // ompl::tools::Profiler::End("RiskZones");
        return state_risk;
    }

    // FCL
    fcl::Transform3f robot_tf;
    robot_tf.setIdentity();
    robot_tf.translate(fcl::Vector3f(state_r2->values[0], state_r2->values[1], 0.0));
    // fcl::Quaternion3f qt0;
    // qt0.fromEuler(0.0, 0.0, 0.0);
    // robot_tf.setQuatRotation(qt0);

    std::shared_ptr<fcl::Cylinderf> cyl0(new fcl::Cylinderf(robot_base_radius_ + 0.2, robot_base_height_));

    fcl::CollisionObjectf cyl0_co(cyl0, robot_tf);
    fcl::CollisionRequestf collision_request;
    fcl::CollisionResultf collision_result;

    fcl::collide(tree_obj_, &cyl0_co, collision_request, collision_result);

    if (collision_result.isCollision())
        state_risk = 10.0; // 15, 30
    else
    {
        std::shared_ptr<fcl::Cylinderf> cyl1(new fcl::Cylinderf(robot_base_radius_ + 0.4, robot_base_height_));
        fcl::CollisionObjectf cyl1_co(cyl1, robot_tf);
        collision_result.clear();

        fcl::collide(tree_obj_, &cyl1_co, collision_request, collision_result);
        if (collision_result.isCollision())
            state_risk = 5.0; // 10, 20
    }

    // ompl::tools::Profiler::End("RiskZones");

    return state_risk;
}

double OmFclStateValidityCheckerR2::checkSocialCostmap(const ob::State *state,
                                                       const ob::SpaceInformationPtr space) const
{
    // ROS_INFO_STREAM("Running social costmap cost objective");

    double state_risk = 1.0;

    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    double mapOriginX = socialCostmap->info.origin.position.x + (socialCostmap->info.width / 2) * socialCostmap->info.resolution;

    double mapOriginY = socialCostmap->info.origin.position.y + (socialCostmap->info.height / 2) * socialCostmap->info.resolution;

    unsigned int nearI = IMapIndex(mapOriginX, socialCostmap->info.width, socialCostmap->info.resolution, state_r2->values[0]);

    unsigned int nearJ = JMapIndex(mapOriginY, socialCostmap->info.height, socialCostmap->info.resolution, state_r2->values[1]);

    if (nearI > socialCostmap->info.width || nearJ > socialCostmap->info.height)
    {
        return state_risk;
    }

    state_risk = socialCostmapValues[mapIndex(socialCostmap->info.width, nearI, nearJ)];

    if (state_risk < 1)
    {
        state_risk = 1.0;
        return state_risk;
    }

    return 2 * state_risk;
}

bool OmFclStateValidityCheckerR2::isValidPoint(const ob::State *state) const
{
    OcTreeNode *result;
    point3d query;
    double node_occupancy;

    // extract the component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *state_r2 = state->as<ob::RealVectorStateSpace::StateType>();

    query.x() = state_r2->values[0];
    query.y() = state_r2->values[1];
    query.z() = 0.0;

    result = octree_->search(query);

    if (result == NULL)
    {
        return false;
    }
    else
    {
        node_occupancy = result->getOccupancy();
        if (node_occupancy <= 0.4)
            return true;
    }
    return false;
}

OmFclStateValidityCheckerR2::~OmFclStateValidityCheckerR2()
{
    delete octree_;
    //    delete tree_;
    //    delete tree_obj_;
}
