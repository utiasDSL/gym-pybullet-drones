#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <math.h>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "custom_interface_gym/msg/traj_msg.hpp"
#include "rrt_path_finder/corridor_finder.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "rrt_path_finder/firi.hpp"
#include "rrt_path_finder/gcopter.hpp"
#include "rrt_path_finder/trajectory.hpp"
#include "rrt_path_finder/geo_utils.hpp"
#include "rrt_path_finder/quickhull.hpp"

using namespace std;
using namespace Eigen;
using namespace pcl;

class PointCloudPlanner : public rclcpp::Node
{
public:
    PointCloudPlanner() : Node("point_cloud_planner"),
                          tf_buffer_(this->get_clock()),
                          tf_listener_(tf_buffer_)
    {
        // Parameters
        this->declare_parameter("plan_rate", 10.0);
        // safety_margin=1.0, search_margin=0.5, max_radius=1.5, sample_range=10.0
        // rrt.setPt(startPt=start_point, endPt=end_point, xl=-5, xh=15, yl=-5, yh=15, zl=0.0, zh=1,
        //      local_range=10, max_iter=1000, sample_portion=0.1, goal_portion=0.05)

        this->declare_parameter("safety_margin", 1.00);
        this->declare_parameter("search_margin", 1.00);
        this->declare_parameter("max_radius", 2.0);
        this->declare_parameter("sensing_range", 6.0);
        this->declare_parameter("local_range",2.0);
        this->declare_parameter("refine_portion", 0.80);
        this->declare_parameter("sample_portion", 0.25);
        this->declare_parameter("goal_portion", 0.05);
        this->declare_parameter("path_find_limit", 5.0);
        this->declare_parameter("max_samples", 10000);
        this->declare_parameter("stop_horizon", 0.5);
        this->declare_parameter("commit_time", 1.0);

        this->declare_parameter("x_l", -75.0);
        this->declare_parameter("x_h", 75.0);
        this->declare_parameter("y_l", -75.0);
        this->declare_parameter("y_h", 75.0);
        this->declare_parameter("z_l", -3.0);
        this->declare_parameter("z_h", 6.0);

        this->declare_parameter("target_x", 0.0);
        this->declare_parameter("target_y", 0.0);
        this->declare_parameter("target_z", 0.0);
        this->declare_parameter("goal_input", true);
        this->declare_parameter("is_limit_vel", true);
        this->declare_parameter("is_limit_acc", true);
        this->declare_parameter("is_print", true);

        // Read parameters
        _planning_rate = this->get_parameter("plan_rate").as_double();
        _safety_margin = this->get_parameter("safety_margin").as_double();
        _search_margin = this->get_parameter("search_margin").as_double();
        _max_radius = this->get_parameter("max_radius").as_double();
        _sensing_range = this->get_parameter("sensing_range").as_double();
        _local_range = this->get_parameter("local_range").as_double();
        _replan_distance = this->get_parameter("sensing_range").as_double();
        _refine_portion = this->get_parameter("refine_portion").as_double();
        _sample_portion = this->get_parameter("sample_portion").as_double();
        _goal_portion = this->get_parameter("goal_portion").as_double();
        _path_find_limit = this->get_parameter("path_find_limit").as_double();
        _max_samples = this->get_parameter("max_samples").as_int();
        _stop_time = this->get_parameter("stop_horizon").as_double();
        _time_commit = this->get_parameter("commit_time").as_double();
        _x_l = this->get_parameter("x_l").as_double();
        _x_h = this->get_parameter("x_h").as_double();
        _y_l = this->get_parameter("y_l").as_double();
        _y_h = this->get_parameter("y_h").as_double();
        _z_l = this->get_parameter("z_l").as_double();
        _z_h = this->get_parameter("z_h").as_double();


        // Set parameters for RRT planner once
        setRRTPlannerParams();

        // Publishers
        // _vis_corridor_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("flight_corridor", 1);
        _vis_rrt_tree_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("_vis_rrt_tree", 1);
        _vis_corridor_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("_viscorridor", 1);
        _vis_rrt_path_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("_vis_rrt_path",1);
        _vis_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("_vis_pcd", 1);
        _vis_mesh_pub = this->create_publisher<visualization_msgs::msg::Marker>("_vis_mesh", 10);
        _vis_edge_pub = this->create_publisher<visualization_msgs::msg::Marker>("_vis_edge", 10);
        _vis_route_pub = this->create_publisher<visualization_msgs::msg::Marker>("_vis_route", 10);
        _vis_waypoint_pub = this->create_publisher<visualization_msgs::msg::Marker>("_vis_waypoints", 10);
        _vis_trajectory_pub = this->create_publisher<visualization_msgs::msg::Marker>("_vis_trajectory", 10);

        // Add the RRT waypoints publisher
        _rrt_waypoints_pub = this->create_publisher<nav_msgs::msg::Path>("rrt_waypoints", 1);
        _rrt_traj_pub = this->create_publisher<custom_interface_gym::msg::TrajMsg>("rrt_trajectory",1);

        // Subscribers
        _obs_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "obs", 1, std::bind(&PointCloudPlanner::rcvObsCallback, this, std::placeholders::_1));
        _dest_pts_sub = this->create_subscription<nav_msgs::msg::Path>(
            "waypoints", 1, std::bind(&PointCloudPlanner::rcvWaypointsCallBack, this, std::placeholders::_1));
        _map_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pcd_gym_pybullet", 1, std::bind(&PointCloudPlanner::rcvPointCloudCallBack, this, std::placeholders::_1));

        // Timer for planning
        _planning_timer = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / _planning_rate),
            std::bind(&PointCloudPlanner::planningCallBack, this));
    };

private:
    // Visualization Publishers
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _vis_corridor_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _vis_rrt_tree_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _vis_rrt_path_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _vis_corridor_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _vis_map_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _vis_mesh_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _vis_edge_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _vis_route_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _vis_waypoint_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _vis_trajectory_pub;

    // RRT waypoints publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _rrt_waypoints_pub;
    rclcpp::Publisher<custom_interface_gym::msg::TrajMsg>::SharedPtr _rrt_traj_pub;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _obs_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _dest_pts_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _map_sub;

    // Timer
    rclcpp::TimerBase::SharedPtr _planning_timer;

    // TF2 buffer and listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Path Planning Parameters
    double _planning_rate, _safety_margin, _search_margin, _max_radius, _sensing_range, _local_range, _replan_distance;
    double _refine_portion, _sample_portion, _goal_portion, _path_find_limit, _stop_time, _time_commit;
    double _x_l, _x_h, _y_l, _y_h, _z_l, _z_h;  // For random map simulation: map boundary
    
    std::vector<Eigen::MatrixX4d> hpolys; // Matrix to store hyperplanes
    std::vector<Eigen::Vector3d> pcd_points; // Datastructure to hold pointcloud points in a vector
    std::chrono::time_point<std::chrono::steady_clock> trajstamp; // timestamp for when trajectory is generated

    int quadratureRes = 16;
    float weightT = 20.0;
    float smoothingEps = 0.01;
    float relcostto1 = 0.00001;
    int _max_samples;
    int _commit_size = 2;
    float threshold = 0.5;
    // RRT Path Planner
    safeRegionRrtStar _rrtPathPlanner;
    gcopter::GCOPTER_PolytopeSFC _gCopter;
    Trajectory<5> _traj;


    // Variables for target position, trajectory, odometry, etc.
    Eigen::Vector3d _start_pos, _end_pos, _start_vel, _start_acc;
    Eigen::Vector3d _commit_target{0.0, 0.0, 0.0};

    Eigen::MatrixXd _path;
    Eigen::VectorXd _radius;
    std::vector<Eigen::Vector3d> _path_vector;
    std::vector<double> _radius_vector;

    nav_msgs::msg::Odometry _odom;
    bool _is_traj_exist = false;
    bool _is_target_arrive = false;
    bool _is_target_receive = false;
    bool _is_has_map = false;

    // ROS 2-compatible callback functions
    void rcvWaypointsCallBack(const nav_msgs::msg::Path::SharedPtr wp_msg)
    {
        if (wp_msg->poses.empty() || wp_msg->poses[0].pose.position.z < 0.0)
            return;

        _end_pos(0) = wp_msg->poses[0].pose.position.x;
        _end_pos(1) = wp_msg->poses[0].pose.position.y;
        _end_pos(2) = wp_msg->poses[0].pose.position.z;

        _is_target_receive = true;
        _is_target_arrive = false;
        _is_traj_exist = false;
        //RCLCPP_WARN(this->get_logger(), "Waypoint received");
    }

    // Initializing rrt parameters
    void setRRTPlannerParams()
    {
        _rrtPathPlanner.setParam(_safety_margin, _search_margin, _max_radius, _sensing_range);
        _rrtPathPlanner.reset();
    }

    // Observation callback (for simulation only)
    void rcvObsCallback(const std_msgs::msg::Float32MultiArray obs_msg)
    {
        _start_pos(0) = obs_msg.data[0];
        _start_pos(1) = obs_msg.data[1];
        _start_pos(2) = obs_msg.data[2];
        
        if(_rrtPathPlanner.getDis(_start_pos, _commit_target) < threshold)
        {
            _is_target_arrive = true;
        }
        //RCLCPP_WARN(this->get_logger(), "Start Pos: %f: %f: %f", _start_pos(0), _start_pos(1), _start_pos(2));
    }

    void rcvPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg)
    {
        if (pointcloud_msg->data.empty())
            return;

        // Transform the point cloud from camera frame to map frame
        sensor_msgs::msg::PointCloud2 cloud_transformed;

        try
        {
            tf_buffer_.transform(*pointcloud_msg, cloud_transformed, "ground_link", tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud_input;
        pcl::fromROSMsg(cloud_transformed, cloud_input);
        //pcl::fromROSMsg(*pointcloud_msg, cloud_input);
        if (cloud_input.points.empty())
            return;

        _is_has_map = true;
        _rrtPathPlanner.setInput(cloud_input);
        for (const auto& point : cloud_input.points)
        {
            Eigen::Vector3d eigen_point(point.x, point.y, point.z);
            pcd_points.push_back(eigen_point);
        }
        _vis_map_pub->publish(cloud_transformed);
        //RCLCPP_WARN(this->get_logger(), "Point Cloud received");
        
    }

    // Function to publish RRT waypoints
    void publishRRTWaypoints(const std::vector<Eigen::Vector3d>& path)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "ground_link";  // Adjust this frame to your use case

        for (const auto& point : path)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "ground_link";
            pose.pose.position.x = point.x();
            pose.pose.position.y = point.y();
            pose.pose.position.z = point.z();
            path_msg.poses.push_back(pose);
        }

        _rrt_waypoints_pub->publish(path_msg);
        //RCLCPP_WARN(this->get_logger(),"rrt path published");
    }


    void convexCover(const Eigen::MatrixXd &path, 
                        const double &range,
                        const double eps)
    {
        Eigen::Vector3d lowCorner(_x_l, _y_l, _z_l);
        Eigen::Vector3d highCorner(_x_h, _y_h, _z_h);
        double progress = _max_radius;
        hpolys.clear();
        int n = int(path.rows());
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
            bd(0, 0) = 1.0;
            bd(1, 0) = -1.0;
            bd(2, 1) = 1.0;
            bd(3, 1) = -1.0;
            bd(4, 2) = 1.0;
            bd(5, 2) = -1.0;
        
        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a(path(0,0), path(0,1), path(0,2));
        Eigen::Vector3d b = a;
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(pcd_points.size());
        for (int i = 1; i < n;)
        {
            Eigen::Vector3d path_point(path(i, 0), path(i,1), path(i,2));
            a = b;
            if ((a - path_point).norm() > progress)
            {
                b = (path_point- a).normalized() * progress + a;
            }
            else
            {
                b = path_point;
                i++;
            }
            bs.emplace_back(b);

            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

            valid_pc.clear();
            for (const Eigen::Vector3d &p : pcd_points)
            {
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(p);
                }
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());

            firi::firi(bd, pc, a, b, hp);

            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                                ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    firi::firi(bd, pc, a, a, gap, 1);
                    hpolys.emplace_back(gap);
                }
            }

            hpolys.emplace_back(hp);
        }

    }

    inline void shortCut()
    {
        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        if (htemp.size() == 1)
            {
                Eigen::MatrixX4d headPoly = htemp.front();
                htemp.insert(htemp.begin(), headPoly);
            }
        hpolys.clear();

        int M = htemp.size();
        Eigen::MatrixX4d hPoly;
        bool overlap;
        std::deque<int> idices;
        idices.push_front(M - 1);
        for (int i = M - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (j < i - 1)
                {
                    overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
                }
                else
                {
                    overlap = true;
                }
                if (overlap)
                {
                    idices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
    }

    void traj_generation(Eigen::MatrixXd path_rrt)
    {
        Eigen::Vector3d front(path_rrt(0,0), path_rrt(0,1), path_rrt(0,2));
        int n = path_rrt.rows();

        Eigen::Vector3d back(path_rrt(n-1,0), path_rrt(n-1, 1), path_rrt(n-1, 2));


        // GCopter parameters
        Eigen::Matrix3d iniState;
        Eigen::Matrix3d finState;
        iniState << front, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
        finState << back, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
        Eigen::VectorXd magnitudeBounds(5);
        Eigen::VectorXd penaltyWeights(5);
        Eigen::VectorXd physicalParams(6);
        std::vector<float> chiVec = {10000, 10000, 10000, 10000, 100000};
        magnitudeBounds(0) = 4.0;
        magnitudeBounds(1) = 2.1;
        magnitudeBounds(2) = 1.05;
        magnitudeBounds(3) = 2.0;
        magnitudeBounds(4) = 12.0;
        penaltyWeights(0) = chiVec[0];
        penaltyWeights(1) = chiVec[1];
        penaltyWeights(2) = chiVec[2];
        penaltyWeights(3) = chiVec[3];
        penaltyWeights(4) = chiVec[4];
        physicalParams(0) = 0.61;
        physicalParams(1) = 9.8;
        physicalParams(2) = 0.70;
        physicalParams(3) = 0.80;
        physicalParams(4) = 0.01;
        physicalParams(5) = 0.0001;
        int quadratureRes = 16;
        float weightT = 20.0;
        float smoothingEps = 0.01;
        float relcostto1 = 0.00001;
        _traj.clear();
        if (!_gCopter.setup(weightT, iniState, finState, hpolys, INFINITY, smoothingEps, quadratureRes, magnitudeBounds, penaltyWeights, physicalParams))
        {
            std::cout<<"gcopter returned false during setup"<<std::endl;
            _is_traj_exist = false;
        }
        if (std::isinf(_gCopter.optimize(_traj, relcostto1)))
        {
            std::cout<<"gcopter optimization cost is infinity"<<std::endl;
            _is_traj_exist = false;
        }
        if (_traj.getPieceNum() > 0)
        {
            std::cout<<"trajectory successfully generated"<<std::endl;
            trajstamp = std::chrono::steady_clock::now();
            _is_traj_exist = true;
        }
    }

    void traj_publish()
    {
        if(_is_traj_exist)
        {
            double elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - trajstamp).count();
            Eigen::Vector3d des_pos = _traj.getPos(elapsed);
            Eigen::Vector3d des_vel = _traj.getVel(elapsed);
            Eigen::Vector3d des_Acc = _traj.getAcc(elapsed);
            Eigen::Vector3d des_jerk = _traj.getJer(elapsed);

            custom_interface_gym::msg::TrajMsg traj_msg;
            traj_msg.header.stamp = rclcpp::Clock().now();
            traj_msg.header.frame_id = "ground_link"; 

            // Set position
            traj_msg.position.x = des_pos.x();
            traj_msg.position.y = des_pos.y();
            traj_msg.position.z = des_pos.z();

            // Set velocity
            traj_msg.velocity.x = des_vel.x();
            traj_msg.velocity.y = des_vel.y();
            traj_msg.velocity.z = des_vel.z();

            // Set acceleration
            traj_msg.acceleration.x = des_Acc.x();
            traj_msg.acceleration.y = des_Acc.y();
            traj_msg.acceleration.z = des_Acc.z();

            // Set jerk
            traj_msg.jerk.x = des_jerk.x();
            traj_msg.jerk.y = des_jerk.y();
            traj_msg.jerk.z = des_jerk.z();

            // Set yaw 
            Eigen::Vector3d direction = des_pos - _start_pos;    // Vector from start_pos to des_pos
            double yaw = std::atan2(direction.y(), direction.x()); // Yaw in radians
            traj_msg.yaw = yaw;

            // Publish the message
            _rrt_traj_pub->publish(traj_msg); // Replace traj_publisher_ with your actual publisher variable
        }
        else
        {
            Eigen::Vector3d des_pos = _start_pos;
            Eigen::Vector3d des_vel(0.0, 0.0, 0.0);
            Eigen::Vector3d des_Acc(0.0, 0.0, 0.0);
            Eigen::Vector3d des_jerk(0.0, 0.0, 0.0);

            custom_interface_gym::msg::TrajMsg traj_msg;
            traj_msg.header.stamp = rclcpp::Clock().now();
            traj_msg.header.frame_id = "ground_link"; 

            // Set position
            traj_msg.position.x = des_pos.x();
            traj_msg.position.y = des_pos.y();
            traj_msg.position.z = des_pos.z();

            // Set velocity
            traj_msg.velocity.x = des_vel.x();
            traj_msg.velocity.y = des_vel.y();
            traj_msg.velocity.z = des_vel.z();

            // Set acceleration
            traj_msg.acceleration.x = des_Acc.x();
            traj_msg.acceleration.y = des_Acc.y();
            traj_msg.acceleration.z = des_Acc.z();

            // Set jerk
            traj_msg.jerk.x = des_jerk.x();
            traj_msg.jerk.y = des_jerk.y();
            traj_msg.jerk.z = des_jerk.z();

            // Set yaw 
            traj_msg.yaw = 0;

            // Publish the message
            _rrt_traj_pub->publish(traj_msg); // Replace traj_publisher_ with your actual publisher variable

        }
    }

    void getCommitTarget(double _commit_time_limit)
    {
        _commit_target = _traj.getPos(_commit_time_limit);
        std::cout<<"[commit target] set to"<<_commit_target[0]<<" : "<<_commit_target[1]<<" : "<<_commit_target[2]<<std::endl;
    }

    // Function to plan the initial trajectory using RRT
    void planInitialTraj()
    {
        _rrtPathPlanner.reset();

        // Set parameters for the RRT planner
        // safety_margin=1.0, search_margin=0.5, max_radius=1.5, sample_range=10.0

        _rrtPathPlanner.setPt(_start_pos, _end_pos, _x_l, _x_h, _y_l, _y_h, _z_l, _z_h,
                             _local_range, _max_samples, _sample_portion, _goal_portion);
        _rrtPathPlanner.SafeRegionExpansion(0.01);
        std::tie(_path, _radius) = _rrtPathPlanner.getPath();
        _path_vector = matrixToVector(_path);

        if (_rrtPathPlanner.getPathExistStatus())
        {
            // Generate trajectory
            std::cout<<"[Initial planning] initial path found"<<std::endl;
            convexCover(_path, 1.0, 1.0e-6);
            shortCut();
            std::cout<<"hpolys size: "<<hpolys.size()<<std::endl;
            traj_generation(_path);
            if(_is_traj_exist)
            {
                getCommitTarget(1.0); // commit_time = 1.0 seconds
                _rrtPathPlanner.resetRoot(_commit_target);
            }                
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No path found in initial trajectory planning");
            _is_traj_exist = false;
        }
        traj_publish();
        visRrt(_rrtPathPlanner.getTree()); 
        visualizePolytope(hpolys);
        visualize(_traj, _path_vector);
    }

    // Function to plan the incremental trajectory
    void planIncrementalTraj()
    {
        if (_rrtPathPlanner.getGlobalNaviStatus())
        {
            visRrt(_rrtPathPlanner.getTree()); 
            return; // No further planning required if global navigation is complete
        }

        if (checkEndOfCommittedPath())
        {
            if (!_rrtPathPlanner.getPathExistStatus())
            {
                RCLCPP_WARN(this->get_logger(), "Reached committed target but no feasible path exists");
                _is_traj_exist = false;
                return;
            }
            else
            {
                // Reset the root of the RRT planner
                // Get the updated path and publish it
                // std::tie(_path, _radius) = _rrtPathPlanner.getPath();
                std::cout<<"[Incremental planner] reached committed target"<<std::endl;
                convexCover(_path, 1.0, 1e-6);
                shortCut();
                traj_generation(_path);
                if(_is_traj_exist)
                {
                    getCommitTarget(1.0);
                    _rrtPathPlanner.resetRoot(_commit_target);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Safe Trajectory could not be generated: Hovering");
                }
                traj_publish();
                _path_vector = matrixToVector(_path);
                _radius_vector = radiusMatrixToVector(_radius);
                // std::cout<<"size of commit path: "<<commit_path.size()<<" size of total path: "<<path_vector.size()<<std::endl;
                visRrt(_rrtPathPlanner.getTree());
            }
        }
        else
        {
            std::cout<<"[Incremental planner] in refine and evaluate loop"<<std::endl;
            // Continue refining and evaluating the path
            _rrtPathPlanner.SafeRegionRefine(0.01);
            _rrtPathPlanner.SafeRegionEvaluate(0.01);

            // Get the updated path and publish it
            if(_rrtPathPlanner.getPathExistStatus())
            {
                std::cout<<"[Incremental planner] in refine and evaluate loop: Path updated"<<std::endl;
                std::tie(_path, _radius) = _rrtPathPlanner.getPath();
                _path_vector = matrixToVector(_path);
                _radius_vector = radiusMatrixToVector(_radius);
                // publishRRTWaypoints(path_vector);
            }
            traj_publish();            
        }
        //RCLCPP_DEBUG(this->get_logger(),"Traj updated");
        visRrt(_rrtPathPlanner.getTree());
        visualizePolytope(hpolys);
        visualize(_traj, _path_vector); 

    }

    // Planning Callback (Core Path Planning Logic)
    void planningCallBack()
    {
        if (!_is_target_receive || !_is_has_map)
        {
            RCLCPP_DEBUG(this->get_logger(), "No target or map received. _is_target_receive: %s, _is_has_map: %s", 
                    _is_target_receive ? "true" : "false", 
                    _is_has_map ? "true" : "false");
            return;
        }
        //RCLCPP_WARN(this->get_logger(),"rrt path planner called");
        if (!_is_traj_exist)
        {
            planInitialTraj();
        }
        else
        {
            planIncrementalTraj();
        }
    }

    bool checkEndOfCommittedPath()
    {
        if(_is_target_arrive)
        {
            _is_target_arrive = false;
            return true;
        }
        else
        {
            return false;
        }
    }











    inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys)
    {
        Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);

        for (size_t id = 0; id < hPolys.size(); id++)
        {
            oldTris = mesh;
            Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
            geo_utils::enumerateVs(hPolys[id], vPoly);

            quickhull::QuickHull<double> tinyQH;
            const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
            const auto &idxBuffer = polyHull.getIndexBuffer();
            int hNum = idxBuffer.size() / 3;

            curTris.resize(3, hNum * 3);
            for (int i = 0; i < hNum * 3; i++)
            {
                curTris.col(i) = vPoly.col(idxBuffer[i]);
            }
            mesh.resize(3, oldTris.cols() + curTris.cols());
            mesh.leftCols(oldTris.cols()) = oldTris;
            mesh.rightCols(curTris.cols()) = curTris;
        }

        // Create markers for mesh and edges
        visualization_msgs::msg::Marker meshMarker;
        visualization_msgs::msg::Marker edgeMarker;

        meshMarker.id = 0;
        meshMarker.header.stamp = rclcpp::Clock().now();
        meshMarker.header.frame_id = "odom";
        meshMarker.pose.orientation.w = 1.0;
        meshMarker.action = visualization_msgs::msg::Marker::ADD;
        meshMarker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        meshMarker.ns = "mesh";
        meshMarker.color.r = 0.0;
        meshMarker.color.g = 0.0;
        meshMarker.color.b = 1.0;
        meshMarker.color.a = 0.15;
        meshMarker.scale.x = 1.0;
        meshMarker.scale.y = 1.0;
        meshMarker.scale.z = 1.0;

        edgeMarker = meshMarker;
        edgeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edgeMarker.ns = "edge";
        edgeMarker.color.r = 0.0;
        edgeMarker.color.g = 1.0;
        edgeMarker.color.b = 1.0;
        edgeMarker.color.a = 1.0;
        edgeMarker.scale.x = 0.02;

        geometry_msgs::msg::Point point;
        int ptnum = mesh.cols();

        for (int i = 0; i < ptnum; i++)
        {
            point.x = mesh(0, i);
            point.y = mesh(1, i);
            point.z = mesh(2, i);
            meshMarker.points.push_back(point);
        }

        for (int i = 0; i < ptnum / 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                point.x = mesh(0, 3 * i + j);
                point.y = mesh(1, 3 * i + j);
                point.z = mesh(2, 3 * i + j);
                edgeMarker.points.push_back(point);
                point.x = mesh(0, 3 * i + (j + 1) % 3);
                point.y = mesh(1, 3 * i + (j + 1) % 3);
                point.z = mesh(2, 3 * i + (j + 1) % 3);
                edgeMarker.points.push_back(point);
            }
        }

        _vis_mesh_pub->publish(meshMarker);
        _vis_edge_pub->publish(edgeMarker);
    }

    void visualize(const Trajectory<5> &traj, const std::vector<Eigen::Vector3d> &route)
    {
        visualization_msgs::msg::Marker routeMarker;
        visualization_msgs::msg::Marker wayPointsMarker;
        visualization_msgs::msg::Marker trajMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
        routeMarker.header.stamp = rclcpp::Clock().now();
        routeMarker.header.frame_id = "odom";
        routeMarker.pose.orientation.w = 1.0;
        routeMarker.action = visualization_msgs::msg::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.0;
        routeMarker.color.g = 0.0;
        routeMarker.color.b = 0.0;
        routeMarker.color.a = 1.0;
        routeMarker.scale.x = 0.1;

        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -1;
        wayPointsMarker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.color.r = 1.0;
        wayPointsMarker.color.g = 0.0;
        wayPointsMarker.color.b = 0.0;
        wayPointsMarker.scale.x = 0.35;
        wayPointsMarker.scale.y = 0.35;
        wayPointsMarker.scale.z = 0.35;

        trajMarker = routeMarker;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.0;
        trajMarker.color.g = 0.5;
        trajMarker.color.b = 1.0;
        trajMarker.scale.x = 0.3;

        if (!route.empty())
        {
            Eigen::Vector3d last;
            for (size_t i = 0; i < route.size(); i++)
            {
                geometry_msgs::msg::Point point;
                if (i > 0)
                {
                    point.x = last(0);
                    point.y = last(1);
                    point.z = last(2);
                    routeMarker.points.push_back(point);
                }
                point.x = route[i][0] ;
                point.y = route[i][1] ;
                point.z = route[i][2] ;
                routeMarker.points.push_back(point);
                last = route[i];
            }

            _vis_route_pub->publish(routeMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            Eigen::MatrixXd wps = traj.getPositions();
            for (int i = 0; i < wps.cols(); i++)
            {
                geometry_msgs::msg::Point point;
                point.x = wps.col(i)(0);
                point.y = wps.col(i)(1);
                point.z = wps.col(i)(2);
                wayPointsMarker.points.push_back(point);
            }

            _vis_waypoint_pub->publish(wayPointsMarker);
        }

        if (traj.getPieceNum() > 0)
        {
            double T = 0.01;
            Eigen::Vector3d lastX = traj.getPos(0.0);
            for (double t = T; t < traj.getTotalDuration(); t += T)
            {
                geometry_msgs::msg::Point point;
                Eigen::Vector3d X = traj.getPos(t);
                point.x = lastX(0);
                point.y = lastX(1);
                point.z = lastX(2);
                trajMarker.points.push_back(point);
                point.x = X(0);
                point.y = X(1);
                point.z = X(2);
                trajMarker.points.push_back(point);
                lastX = X;
            }
            _vis_trajectory_pub->publish(trajMarker);
        }
    }


    void visRrt(const std::vector<NodePtr>& nodes)
    {
        visualization_msgs::msg::MarkerArray tree_markers;
        int marker_id = 0;

        // Get the tree from the RRT planner
        std::vector<NodePtr> nodeList = _rrtPathPlanner.getTree();

        // Loop through all the nodes in the tree
        for (const auto &node : nodeList) {
            if (node->preNode_ptr != nullptr) { // Only visualize branches (paths)
                visualization_msgs::msg::Marker branch_marker;
                
                // Marker properties
                branch_marker.header.frame_id = "ground_link";
                branch_marker.header.stamp = this->get_clock()->now();
                branch_marker.ns = "rrt_branches";
                branch_marker.id = marker_id++;
                branch_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                branch_marker.action = visualization_msgs::msg::Marker::ADD;

                // Define start and end points for the branch
                geometry_msgs::msg::Point start_point;
                start_point.x = node->coord[0];
                start_point.y = node->coord[1];
                start_point.z = node->coord[2];

                geometry_msgs::msg::Point end_point;
                end_point.x = node->preNode_ptr->coord[0];
                end_point.y = node->preNode_ptr->coord[1];
                end_point.z = node->preNode_ptr->coord[2];

                branch_marker.points.push_back(start_point);
                branch_marker.points.push_back(end_point);

                // Set branch properties: scale, color
                branch_marker.scale.x = 0.01; // Line width
                branch_marker.color.a = 0.8; // Transparency
                branch_marker.color.r = 0.0; // Red
                branch_marker.color.g = 0.0; // Green
                branch_marker.color.b = 1.0; // Blue (for branches)

                // Add the marker to the MarkerArray
                tree_markers.markers.push_back(branch_marker);
            }
        }

        // Publish the MarkerArray
        _vis_rrt_tree_pub->publish(tree_markers);

    }
    void publishCorridorVisualization(const std::vector<Eigen::Vector3d>& path, const std::vector<double>& radii)
    {
        visualization_msgs::msg::MarkerArray corridor_markers;

        for (size_t i = 0; i < path.size(); ++i)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "ground_link";
            marker.header.stamp = this->now();
            marker.ns = "corridor";
            marker.id = static_cast<int>(i);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set position of the marker
            marker.pose.position.x = path[i].x();
            marker.pose.position.y = path[i].y();
            marker.pose.position.z = path[i].z();

            // Set scale (diameter based on radius)
            double diameter = 2.0 * radii[i]; // Radius to diameter
            marker.scale.x = diameter;
            marker.scale.y = diameter;
            marker.scale.z = diameter;

            // Set color and transparency
            marker.color.a = 0.5;  // Transparency
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            corridor_markers.markers.push_back(marker);
        }

        _vis_corridor_pub->publish(corridor_markers);
    }

    std::vector<Eigen::Vector3d> matrixToVector(const Eigen::MatrixXd& path_matrix)
    {
        std::vector<Eigen::Vector3d> path_vector;
        for (int i = 0; i < path_matrix.rows(); ++i)
        {
            Eigen::Vector3d point;
            point.x() = path_matrix(i, 0);
            point.y() = path_matrix(i, 1);
            point.z() = path_matrix(i, 2);
            path_vector.push_back(point);
        }
        return path_vector;
    }

    std::vector<double> radiusMatrixToVector(const Eigen::Matrix<double, -1, 1>& eigen_matrix)
    {
        std::vector<double> vec(eigen_matrix.data(), eigen_matrix.data() + eigen_matrix.size());
        return vec;
    }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPlanner>());
    rclcpp::shutdown();
    return 0;
}
