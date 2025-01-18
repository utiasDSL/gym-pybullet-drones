#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <math.h>
#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "custom_interface_gym/msg/traj_msg.hpp"
#include "custom_interface_gym/msg/des_trajectory.hpp"
#include "rrt_path_finder/corridor_finder.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "rrt_path_finder/firi.hpp"
#include "rrt_path_finder/gcopter.hpp"
#include "rrt_path_finder/trajectory.hpp"
#include "rrt_path_finder/geo_utils.hpp"
#include "rrt_path_finder/quickhull.hpp"
#include "rrt_path_finder/voxel_map.hpp"

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
        // safety_margin=1.0, search_margin=0.5, max_radius=1.5, sample_range=10.0
        // rrt.setPt(startPt=start_point, endPt=end_point, xl=-5, xh=15, yl=-5, yh=15, zl=0.0, zh=1,
        //      max_iter=1000, sample_portion=0.1, goal_portion=0.05)

        this->declare_parameter("safety_margin", 0.5);
        this->declare_parameter("search_margin", 0.4);
        this->declare_parameter("max_radius", 2.0);
        this->declare_parameter("sample_range", 20.0);
        this->declare_parameter("refine_portion", 0.80);
        this->declare_parameter("sample_portion", 0.25);
        this->declare_parameter("goal_portion", 0.05);
        this->declare_parameter("path_find_limit", 5.0);
        this->declare_parameter("max_samples", 10000);
        this->declare_parameter("stop_horizon", 0.5);
        this->declare_parameter("commit_time", 1.0);

        this->declare_parameter("x_l", -70.0);
        this->declare_parameter("x_h", 70.0);
        this->declare_parameter("y_l", -70.0);
        this->declare_parameter("y_h", 70.0);
        this->declare_parameter("z_l", -0.0);
        this->declare_parameter("z_l2", -3.0);

        this->declare_parameter("z_h", 3.5);

        this->declare_parameter("target_x", 0.0);
        this->declare_parameter("target_y", 0.0);
        this->declare_parameter("target_z", 0.0);
        this->declare_parameter("goal_input", true);
        this->declare_parameter("is_limit_vel", true);
        this->declare_parameter("is_limit_acc", true);
        this->declare_parameter("is_print", true);

        // Read parameters
        _safety_margin = this->get_parameter("safety_margin").as_double();
        _search_margin = this->get_parameter("search_margin").as_double();
        _max_radius = this->get_parameter("max_radius").as_double();
        _sample_range = this->get_parameter("sample_range").as_double();
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
        _z_l2 = this->get_parameter("z_l2").as_double();
        Eigen::Vector3i xyz((_x_h-_x_l)/voxelWidth, (_y_h-_y_l)/voxelWidth, (_z_h-_z_l2)/voxelWidth);
        Eigen::Vector3d offset(_x_l, _y_l, _z_l2);
        V_map = voxel_map::VoxelMap(xyz, offset, voxelWidth);
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
        _vis_commit_target = this->create_publisher<visualization_msgs::msg::Marker>("_vis_commit_target", 10);

        _vis_trajectory_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("_vis_trajectory", 10);

        // Add the RRT waypoints publisher
        _rrt_waypoints_pub = this->create_publisher<nav_msgs::msg::Path>("rrt_waypoints", 1);
        // _rrt_traj_pub = this->create_publisher<custom_interface_gym::msg::TrajMsg>("rrt_command",1);
        _rrt_des_traj_pub = this->create_publisher<custom_interface_gym::msg::DesTrajectory>("des_trajectory",10);

        // Subscribers
        // _obs_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        // "obs", 1, std::bind(&PointCloudPlanner::rcvObsCallback, this, std::placeholders::_1));

        _dest_pts_sub = this->create_subscription<nav_msgs::msg::Path>(
            "waypoints", 1, std::bind(&PointCloudPlanner::rcvWaypointsCallBack, this, std::placeholders::_1));
        _map_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "pcd_gym_pybullet", 1, std::bind(&PointCloudPlanner::rcvPointCloudCallBack, this, std::placeholders::_1));
        _odometry_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&PointCloudPlanner::rcvOdomCallback, this, std::placeholders::_1));

        // Timer for planning
        _planning_timer = this->create_wall_timer(
            std::chrono::duration<double>(0.1),
            std::bind(&PointCloudPlanner::planningCallBack, this));
    };

private:
    // Visualization Publishers
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _vis_corridor_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _vis_rrt_tree_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _vis_rrt_path_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _vis_corridor_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _vis_commit_target;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _vis_map_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _vis_mesh_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _vis_edge_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _vis_trajectory_pub;

    // RRT waypoints publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _rrt_waypoints_pub;
    // rclcpp::Publisher<custom_interface_gym::msg::TrajMsg>::SharedPtr _rrt_traj_pub;
    rclcpp::Publisher<custom_interface_gym::msg::DesTrajectory>::SharedPtr _rrt_des_traj_pub;

    // Subscribers
    // rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _obs_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _dest_pts_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _map_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odometry_sub;

    // Timer
    rclcpp::TimerBase::SharedPtr _planning_timer;

    // TF2 buffer and listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Path Planning Parameters
    double _planning_rate, _safety_margin, _search_margin, _max_radius, _sample_range, _replan_distance;
    double _refine_portion, _sample_portion, _goal_portion, _path_find_limit, _stop_time, _time_commit;
    double _x_l, _x_h, _y_l, _y_h, _z_l, _z_h, _z_l2;  // For random map simulation: map boundary
    
    std::vector<Eigen::MatrixX4d> hpolys; // Matrix to store hyperplanes
    std::vector<Eigen::Vector3d> pcd_points; // Datastructure to hold pointcloud points in a vector
    std::vector<Eigen::Vector3d> corridor_points; // vector for storing points for which corridor is contructed
    
    rclcpp::Time trajstamp;
    rclcpp::Time odom_time;
    // rclcpp::Time start_traj_time, end_traj_time;
    rclcpp::Time commit_target_time;
    std::chrono::time_point<std::chrono::steady_clock> obsvstamp = std::chrono::steady_clock::now();
    int quadratureRes = 16;
    float weightT = 20.0;
    float smoothingEps = 0.01;
    float relcostto1 = 0.00001;
    int _max_samples;
    double _commit_distance = 10.0;
    double max_vel = 1.0;
    float threshold = 0.8;
    int trajectory_id = 0;
    int order = 5;
    double convexCoverRange = 0.1;
    // RRT Path Planner
    safeRegionRrtStar _rrtPathPlanner;
    gcopter::GCOPTER_PolytopeSFC _gCopter;
    Trajectory<5> _traj;
    voxel_map::VoxelMap V_map;
    int max_iter=100000;
    float voxelWidth = 0.3;
    float dilateRadius = 0.6;
    float leafsize = 0.4;
    // Variables for target position, trajectory, odometry, etc.
    Eigen::Vector3d _start_pos, _end_pos, _start_vel, _last_vel{0.0, 0.0, 0.0}, _start_acc;
    Eigen::Vector3d _commit_target{0.0, 0.0, 0.0}, _corridor_end_pos;
    Eigen::Vector3d temp_end_point{0.0, 0.0, 0.0};

    Eigen::MatrixXd _path;
    Eigen::VectorXd _radius;
    std::vector<Eigen::Vector3d> _path_vector;
    std::vector<double> _radius_vector;

    nav_msgs::msg::Odometry _odom;
    bool _is_traj_exist = false;
    bool _is_target_arrive = false;
    bool _is_target_receive = false;
    bool _is_has_map = false;
    bool _is_complete = false;


    // ROS 2-compatible callback functions
    void rcvWaypointsCallBack(const nav_msgs::msg::Path::SharedPtr wp_msg)
    {
        if(_is_target_receive) return;
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
        _rrtPathPlanner.setParam(_safety_margin, _search_margin, _max_radius, _sample_range);
        _rrtPathPlanner.reset();
    }

    void rcvOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        _odom = *msg;
        _start_pos[0] = _odom.pose.pose.position.x;
        _start_pos[1] = _odom.pose.pose.position.y;
        _start_pos[2] = _odom.pose.pose.position.z;

        _start_vel[0] = _odom.twist.twist.linear.x;
        _start_vel[1] = _odom.twist.twist.linear.y;
        _start_vel[2] = _odom.twist.twist.linear.z;
        // odom_time = std::chrono::steady_clock::time_point(
        //     std::chrono::seconds(_odom.header.stamp.sec) +
        //     std::chrono::nanoseconds(_odom.header.stamp.nanosec)
        // );

        odom_time = rclcpp::Time(_odom.header.stamp.sec, _odom.header.stamp.nanosec);
        if(_rrtPathPlanner.getDis(_start_pos, _commit_target) < threshold)
        {
            // obsvstamp = std::chrono::steady_clock::now();
            _is_target_arrive = true;
            commit_target_time = rclcpp::Clock().now();
            // end_traj_time = rclcpp::Clock().now();
        }
        else
        {
            _is_target_arrive = false;

            // std::cout<<"[commit debug] distance to commit target: "<<_rrtPathPlanner.getDis(_start_pos, _commit_target)<<std::endl;
            // std::cout<<"[commit debug] distance to endgoal: "<<_rrtPathPlanner.getDis(_start_pos, _end_pos)<<std::endl;
        }
        if(_rrtPathPlanner.getDis(_start_pos, _end_pos) < _safety_margin)
        {
            _is_complete = true;   
        }
        checkSafeTrajectory();
        // RCLCPP_WARN(this->get_logger(), "Received odometry: position(x: %.2f, y: %.2f, z: %.2f)",
               // _odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose);
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
        // std::cout<<"size of subscribed pcd: "<<cloud_input.points.size()<<std::endl;
        pcd_points.clear();
        V_map.reset();
        int i = 0;
        for (const auto &point : cloud_input.points) {
                // Check for invalid points (NaN or Inf)
                if (std::isnan(point.x) || std::isinf(point.x) ||
                    std::isnan(point.y) || std::isinf(point.y) ||
                    std::isnan(point.z) || std::isinf(point.z)) {
                    continue;
                }
                i++;
                // Mark voxel as occupied
                V_map.setOccupied(Eigen::Vector3d(point.x, point.y, point.z));
            }
        int dilateSteps = std::ceil(dilateRadius / V_map.getScale());
        V_map.dilate(dilateSteps);
        V_map.getSurf(pcd_points);
        // std::cout<<"points inserted in V_map: "<<i<<std::endl;
        // std::cout<<" pcd point size after getsurf(): "<<pcd_points.size()<<std::endl;

        pcl::PointCloud<pcl::PointXYZ>::Ptr V_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        // std::cout<<"pcd_points size: "<<pcd_points.size()<<std::endl;
        for(auto point : pcd_points)
        {
            pcl::PointXYZ pcd_point;
            pcd_point.x = point(0);
            pcd_point.y = point(1);
            pcd_point.z = point(2);
            V_map_cloud->points.push_back(pcd_point);

        }

        _rrtPathPlanner.setInput(*V_map_cloud);
        _rrtPathPlanner.setInputforCollision(cloud_input);
            sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(*V_map_cloud, filtered_cloud_msg);
        filtered_cloud_msg.header.frame_id = "ground_link"; // Set appropriate frame ID
        filtered_cloud_msg.header.stamp = this->get_clock()->now();
        _vis_map_pub->publish(filtered_cloud_msg);
        
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

    void getCorridorPoints()
    {
        corridor_points.clear();
        float r = 0;
        temp_end_point = _corridor_end_pos;
        if(_rrtPathPlanner.getDis(_start_pos, _end_pos) < _commit_distance)
        {
            corridor_points = _path_vector;
            _corridor_end_pos = _end_pos;
            return;
        }
        else
        {
            for(int i=0; i<_path_vector.size(); i++)
            {
                if(r < _commit_distance && _radius[i] > _safety_margin)
                {
                    corridor_points.push_back(_path_vector[i]);
                    r += _radius[i];
                }
            }
            _corridor_end_pos = corridor_points[corridor_points.size()-1];
        }
    }

    void convexCover(const std::vector<Eigen::Vector3d> &path, 
                        const double &range,
                        const double eps)
    {
        Eigen::Vector3d lowCorner(_x_l, _y_l, _z_l2);
        Eigen::Vector3d highCorner(_x_h, _y_h, _z_h);
        hpolys.clear();
        int n = int(path.size());
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
            bd(0, 0) = 1.0;
            bd(1, 0) = -1.0;
            bd(2, 1) = 1.0;
            bd(3, 1) = -1.0;
            bd(4, 2) = 1.0;
            bd(5, 2) = -1.0;
        
        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a(path[0][0], path[0][1], path[0][2]);
        Eigen::Vector3d b = a;
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(pcd_points.size());
        for (int i = 1; i < n;)
        {
            Eigen::Vector3d path_point = path[i];
            a = b;
            b = path_point;
            i++;
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

    void traj_generation(Eigen::Vector3d ini_pose, Eigen::Vector3d ini_vel)
    {

        // GCopter parameters
        Eigen::Matrix3d iniState;
        Eigen::Matrix3d finState;
        // start_traj_time = rclcpp::Clock().now();
        iniState << ini_pose, ini_vel, Eigen::Vector3d::Zero();
        finState << _corridor_end_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
        Eigen::VectorXd magnitudeBounds(5);
        Eigen::VectorXd penaltyWeights(5);
        Eigen::VectorXd physicalParams(6);
        std::vector<float> chiVec = {10000, 10000, 10000, 10000, 100000};
        magnitudeBounds(0) = max_vel;
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
        float relcostto1 = 1e-3;
        _traj.clear();
        auto t1 = std::chrono::steady_clock::now();
        std::cout<<"Number of hyperplanes is : "<<hpolys.size()<<endl;
        std::cout<<"Trajectory length : "<<_traj.getPieceNum()<<endl;
        if (!_gCopter.setup(weightT, iniState, finState, hpolys, INFINITY, smoothingEps, quadratureRes, magnitudeBounds, penaltyWeights, physicalParams))
        {
            std::cout<<"gcopter returned false during setup, traj exist set to false"<<std::endl;
            _is_traj_exist = false;
            custom_interface_gym::msg::DesTrajectory des_traj_msg;
            des_traj_msg.header.stamp = rclcpp::Clock().now();
            des_traj_msg.header.frame_id = "ground_link";
            des_traj_msg.action = des_traj_msg.ACTION_WARN_IMPOSSIBLE;
            _rrt_des_traj_pub->publish(des_traj_msg);
            return;
        }
        if (std::isinf(_gCopter.optimize(_traj, relcostto1)))
        {
            std::cout<<"gcopter optimization cost is infinity, traj exist set to false"<<std::endl;
            _is_traj_exist = false;
            custom_interface_gym::msg::DesTrajectory des_traj_msg;
            des_traj_msg.header.stamp = rclcpp::Clock().now();
            des_traj_msg.header.frame_id = "ground_link";
            des_traj_msg.action = des_traj_msg.ACTION_WARN_IMPOSSIBLE;
            _rrt_des_traj_pub->publish(des_traj_msg);
            return;
        }
        if (_traj.getPieceNum() > 0)
        {
            // trajstamp = std::chrono::steady_clock::now();
            trajstamp = rclcpp::Clock().now();
            auto t2 = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()*0.001;

            std::cout<<"[GCOPTER Debug] trajectory successfully generated, traj exist set to true. time taken for trajectory generation: "<<elapsed<<std::endl;

            _is_traj_exist = true;
            custom_interface_gym::msg::DesTrajectory des_traj_msg;
            des_traj_msg.header.stamp = rclcpp::Clock().now();
            des_traj_msg.header.frame_id = "ground_link";
            des_traj_msg.trajectory_id = trajectory_id++;
            des_traj_msg.action = des_traj_msg.ACTION_ADD;
            des_traj_msg.num_order = order;
            des_traj_msg.num_segment = _traj.getPieceNum();
            Eigen::VectorXd durations = _traj.getDurations();
            std::vector<double> durations_vec(durations.data(), durations.data() + durations.size());
            auto coefficient_mat = _traj.getCoefficientMatrices();
            // for (auto mat : coefficient_mat)
            // {
            //     std::cout<<"######## mat #########"<<std::endl;
            //     std::cout<<mat<<std::endl;
            // }
            for(int i=0; i<_traj.getPieceNum(); i++)
            {
                des_traj_msg.duration_vector.push_back(durations_vec[i]);
                for(int j=0; j<coefficient_mat[i].rows(); j++)
                {
                    for(int k=0; k<coefficient_mat[i].cols(); k++)
                    {
                        des_traj_msg.matrices_flat.push_back(coefficient_mat[i](j, k));
                        // only for debugging 
                    }
                }
            }
            des_traj_msg.debug_info = "trajectory_id: "+std::to_string(trajectory_id-1);
            _rrt_des_traj_pub->publish(des_traj_msg);            
            // std::cout<<std::endl;
            return;
        }
    }

    void getCommitTarget()
    {
        _commit_target = _traj.getPos(0.75 * _traj.getTotalDuration());
        // if(_rrtPathPlanner.getDis(_start_pos, _commit_target) > _rrtPathPlanner.getDis(_start_pos, _end_pos)) _commit_target = _end_pos;
        // std::cout<<"[commit target] set to"<<_commit_target[0]<<" : "<<_commit_target[1]<<" : "<<_commit_target[2]<<std::endl;
    }

    // Function to plan the initial trajectory using RRT
    void planInitialTraj()
    {
        _rrtPathPlanner.reset();
        _rrtPathPlanner.setPt(_start_pos, _end_pos, _x_l, _x_h, _y_l, _y_h, _z_l, _z_h,
                             _sample_range, _max_samples, _sample_portion, _goal_portion);
        _rrtPathPlanner.SafeRegionExpansion(0.05);
        std::tie(_path, _radius) = _rrtPathPlanner.getPath();

        if (_rrtPathPlanner.getPathExistStatus())
        {
            // Generate trajectory
            std::cout<<"[Initial planning] initial path found"<<std::endl;
            _path_vector = matrixToVector(_path);
            getCorridorPoints();
            auto t1 = std::chrono::steady_clock::now();
            convexCover(corridor_points, convexCoverRange, 1.0e-6);
            shortCut();
            auto t2 = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()*0.001;
            std::cout<<"[Initial planning] time taken in corridor generation "<<elapsed<<std::endl;

            traj_generation(_start_pos, _start_vel);
            if(_is_traj_exist)
            {
                getCommitTarget(); // commit_time = 1.0 seconds
                _rrtPathPlanner.resetRoot(_commit_target);
                visualizePolytope(hpolys);
                visualizeTrajectory(_traj);
            }      

        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No path found in initial trajectory planning");
            _is_traj_exist = false;
            custom_interface_gym::msg::DesTrajectory des_traj_msg;
            des_traj_msg.header.stamp = rclcpp::Clock().now();
            des_traj_msg.header.frame_id = "ground_link";
            des_traj_msg.action = des_traj_msg.ACTION_WARN_IMPOSSIBLE;
            _rrt_des_traj_pub->publish(des_traj_msg);

        }
        visRrt(_rrtPathPlanner.getTree()); 
        visRRTPath(_path);
        visCommitTarget();
    }

    void planIncrementalTraj()
    {
        if (_rrtPathPlanner.getGlobalNaviStatus())
        {
            RCLCPP_WARN(this->get_logger(), "Almost reached final goal");
            return; // No further planning required if global navigation is complete
        }

        if (checkEndOfCommittedPath())
        {
            if (!_rrtPathPlanner.getPathExistStatus())
            {
                RCLCPP_WARN(this->get_logger(), "Reached committed target but no feasible path exists");
                _is_traj_exist = false;
                custom_interface_gym::msg::DesTrajectory des_traj_msg;
                des_traj_msg.header.stamp = rclcpp::Clock().now();
                des_traj_msg.header.frame_id = "ground_link";
                des_traj_msg.action = des_traj_msg.ACTION_WARN_IMPOSSIBLE;
                _rrt_des_traj_pub->publish(des_traj_msg);
                return;
            }
            else
            {
                // Reset the root of the RRT planner
                // Get the updated path and publish it
                std::tie(_path, _radius) = _rrtPathPlanner.getPath();
                _path_vector = matrixToVector(_path);
                getCorridorPoints();
                auto t1 = std::chrono::steady_clock::now();

                convexCover(corridor_points, convexCoverRange, 1.0e-6);
                shortCut();
                auto t2 = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()*0.001;

                Eigen::Vector3d start_position_new, start_velocity_new, start_acceleration_new;
                rclcpp::Duration del_t = commit_target_time - trajstamp;
                rclcpp::Duration del_time = rclcpp::Duration::from_seconds(0.2); 
                // rclcpp::Duration del_time = end_traj_time - start_traj_time;
                auto new_time = del_t + del_time; 
                // auto new_time = del_time;

                if(_traj.getTotalDuration() >new_time.seconds())
                {   
                    cout<<"Total Time is : "<<_traj.getTotalDuration()<<endl;  
                    cout<<"New time is : "<<new_time.seconds()<<endl;
                    cout<<"New Pose is : "<< _traj.getPos(new_time.seconds())[0]<<' '<< _traj.getPos(new_time.seconds())[1]<<' '<< _traj.getPos(new_time.seconds())[2]<<endl;
                    start_position_new = _traj.getPos(new_time.seconds());
                    start_velocity_new = _traj.getVel(new_time.seconds());
                    start_acceleration_new = _traj.getAcc(new_time.seconds());
                }
                else
                {
                    cout<<"Exceeded where it was supposed to stop before"<<endl;
                    cout<<temp_end_point[0]<<' '<<temp_end_point[1]<<' '<<temp_end_point[2]<<endl;
                    start_position_new = temp_end_point;
                    start_velocity_new[0] = 0;
                    start_velocity_new[1] = 0;
                    start_velocity_new[2] = 0;
                }

                std::cout<<"[Incremental planner] reached committed target, time taken in corridor generation = "<<elapsed<<std::endl;
                traj_generation(start_position_new, start_velocity_new);
                if(_is_traj_exist)
                {
                    getCommitTarget();
                    _rrtPathPlanner.resetRoot(_commit_target);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Safe Trajectory could not be generated: Hovering");
                    custom_interface_gym::msg::DesTrajectory des_traj_msg;
                    des_traj_msg.header.stamp = rclcpp::Clock().now();
                    des_traj_msg.header.frame_id = "ground_link";
                    des_traj_msg.action = des_traj_msg.ACTION_WARN_IMPOSSIBLE;
                    _rrt_des_traj_pub->publish(des_traj_msg);
                }
                _path_vector = matrixToVector(_path);
                _radius_vector = radiusMatrixToVector(_radius);
            }
        }
        else
        {
            // std::cout<<"[Incremental planner] in refine and evaluate loop"<<std::endl;
            auto time_start_ref = std::chrono::steady_clock::now();
            // Continue refining and evaluating the path
            _rrtPathPlanner.SafeRegionRefine(0.08);
            _rrtPathPlanner.SafeRegionEvaluate(0.02);
            auto time_end_ref = std::chrono::steady_clock::now();

            // Get the updated path and publish it
            if(_rrtPathPlanner.getPathExistStatus())
            {
                // std::cout<<"[Incremental planner] in refine and evaluate loop: Path updated"<<std::endl;
                std::tie(_path, _radius) = _rrtPathPlanner.getPath();
                // convexCover(_path, 1.0, _safety_margin, 1e-6);
                // shortCut();
                _path_vector = matrixToVector(_path);
                // publishRRTWaypoints(path_vector);
            }
            double elapsed_ms = std::chrono::duration_cast<std::chrono::seconds>(time_end_ref - time_start_ref).count();
            // std::cout<<"[incremental planner] time duration: "<<elapsed_ms<<std::endl;
            visualizePolytope(hpolys);
            visualizeTrajectory(_traj);
        }
        //RCLCPP_DEBUG(this->get_logger(),"Traj updated");
        visRrt(_rrtPathPlanner.getTree());
        visRRTPath(_path); 
        visCommitTarget();

    }

    // Planning Callback (Core Path Planning Logic)
    void planningCallBack()
    {
        if(_is_complete)
        {
            RCLCPP_WARN(this->get_logger(),"Reached GOAL");
            custom_interface_gym::msg::DesTrajectory des_traj_msg;
            des_traj_msg.header.stamp = rclcpp::Clock().now();
            des_traj_msg.header.frame_id = "ground_link";
            des_traj_msg.action = des_traj_msg.ACTION_WARN_FINAL;
            _rrt_des_traj_pub->publish(des_traj_msg);
            return;
        }
        if (!_is_target_receive || !_is_has_map)
        {
            RCLCPP_DEBUG(this->get_logger(), "No target or map received. _is_target_receive: %s, _is_has_map: %s", 
                    _is_target_receive ? "true" : "false", 
                    _is_has_map ? "true" : "false");
            return;
        }
        //RCLCPP_WARN(this->get_logger(),"rrt path planner called");
        if (_is_traj_exist == false)
        {
            // std::cout<<"[planning callback] Iniial planner: traj_exist = "<<_is_traj_exist<<std::endl;
            planInitialTraj();
        }
        else
        {
            auto time_start_ref = std::chrono::steady_clock::now();
            planIncrementalTraj();
            auto time_end_ref = std::chrono::steady_clock::now();

            double elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(time_end_ref - time_start_ref).count()*0.001;
            // std::cout<<"[planning callback] time taken in incremental planning: "<<elapsed_ms<<std::endl;
        }   
        
    }

    bool checkSafeTrajectory()
    {   
        auto delta_t = (odom_time - trajstamp).seconds();
        // std::cout<<"[safety debug] delta T: "<<delta_t<<std::endl;

        if(!_is_traj_exist) return false;
        double T = max(0.0, delta_t);
        // std::cout<<"[safety debug] checking safe trajectory, delta t = "<<T<<std::endl;
        for(double t = T; t < T+1.0; t += 0.01)
        {
            auto pos_t = _traj.getPos(t);
            if(_rrtPathPlanner.checkTrajPtCol(pos_t))
            {
                RCLCPP_WARN(this->get_logger(), "UNSAFE Trajectory: reverting to new generation");
                _is_traj_exist = false;
                return false;
            }
        }
        return true;
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
        visualization_msgs::msg::Marker mesh_marker;
        mesh_marker.header.frame_id = "ground_link";  // Replace with your desired frame ID
        mesh_marker.header.stamp = rclcpp::Clock().now();
        mesh_marker.ns = "polytope";
        mesh_marker.id = 0;  // Unique ID for the mesh
        mesh_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;  // Type: TRIANGLE_LIST
        mesh_marker.action = visualization_msgs::msg::Marker::ADD;

        mesh_marker.scale.x = 1.0;
        mesh_marker.scale.y = 1.0;
        mesh_marker.scale.z = 1.0;

        mesh_marker.color.r = 0.0f;  // Red
        mesh_marker.color.g = 1.0f;  // Green
        mesh_marker.color.b = 0.0f;  // Blue
        mesh_marker.color.a = 0.8f;  // Transparency

        // Marker for the wireframe (edges)
        visualization_msgs::msg::Marker edges_marker;
        edges_marker.header.frame_id = "ground_link";  // Same frame ID
        edges_marker.header.stamp = rclcpp::Clock().now();
        edges_marker.ns = "polytope_edges";
        edges_marker.id = 1;  // Unique ID for the edges
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;  // Type: LINE_LIST
        edges_marker.action = visualization_msgs::msg::Marker::ADD;

        edges_marker.scale.x = 0.02;  // Line thickness

        edges_marker.color.r = 1.0f;  // Red for edges
        edges_marker.color.g = 1.0f;  // Green for edges
        edges_marker.color.b = 1.0f;  // Blue for edges
        edges_marker.color.a = 1.0f;  // Full opacity

        // Iterate over polytopes
        for (const auto &hPoly : hPolys) {
            // Enumerate vertices of the polytope from half-space representation (Ax <= b)
            Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
            geo_utils::enumerateVs(hPoly, vPoly);  // Assumes `enumerateVs` computes vertices

            // Use QuickHull to compute the convex hull
            quickhull::QuickHull<double> tinyQH;
            const auto polyHull = tinyQH.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
            const auto &idxBuffer = polyHull.getIndexBuffer();

            // Add triangles to the mesh marker
            for (size_t i = 0; i < idxBuffer.size(); i += 3) {
                geometry_msgs::msg::Point p1, p2, p3;

                // Vertex 1
                p1.x = vPoly(0, idxBuffer[i]);
                p1.y = vPoly(1, idxBuffer[i]);
                p1.z = vPoly(2, idxBuffer[i]);

                // Vertex 2
                p2.x = vPoly(0, idxBuffer[i + 1]);
                p2.y = vPoly(1, idxBuffer[i + 1]);
                p2.z = vPoly(2, idxBuffer[i + 1]);

                // Vertex 3
                p3.x = vPoly(0, idxBuffer[i + 2]);
                p3.y = vPoly(1, idxBuffer[i + 2]);
                p3.z = vPoly(2, idxBuffer[i + 2]);

                // Add points to the mesh marker
                mesh_marker.points.push_back(p1);
                mesh_marker.points.push_back(p2);
                mesh_marker.points.push_back(p3);

                // Add edges to the wireframe marker
                edges_marker.points.push_back(p1);
                edges_marker.points.push_back(p2);

                edges_marker.points.push_back(p2);
                edges_marker.points.push_back(p3);

                edges_marker.points.push_back(p3);
                edges_marker.points.push_back(p1);
            }
        }

        // Publish both markers
        _vis_mesh_pub->publish(mesh_marker);  // Publisher for the mesh
        _vis_edge_pub->publish(edges_marker);  // Publisher for the edges
    }

    void visualizeTrajectory(const Trajectory<5> &traj)
    {
        sensor_msgs::msg::PointCloud2 trajectory_cloud;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr traj_points(new pcl::PointCloud<pcl::PointXYZRGBA>());


        double T = 0.01; // Sampling interval
        Eigen::Vector3d lastX = traj.getPos(0.0);

        for (double t = T; t < traj.getTotalDuration(); t += T) {
            Eigen::Vector3d X = traj.getPos(t);
            pcl::PointXYZRGBA point;

            // Add the current point to the trajectory point cloud
            point.x = X(0);
            point.y = X(1);
            point.z = X(2);
            point.r = 0;
            point.g = 255;
            point.b = 0;
            point.a = 255;
            traj_points->points.push_back(point);
        }
        pcl::toROSMsg(*traj_points, trajectory_cloud);

        // Set header information
        trajectory_cloud.header.frame_id = "ground_link";  // Replace "map" with your frame ID
        trajectory_cloud.header.stamp = rclcpp::Clock().now();
        _vis_trajectory_pub->publish(trajectory_cloud);

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

    void visRRTPath(const Eigen::MatrixXd& path_matrix)
    {
        visualization_msgs::msg::MarkerArray path_visualizer;
        int marker_id = 0;
        
        visualization_msgs::msg::Marker point_vis_og;
            point_vis_og.header.frame_id = "ground_link";
            point_vis_og.header.stamp = this->get_clock()->now();
            point_vis_og.ns = "rrt_path";
            point_vis_og.id = marker_id++;
            point_vis_og.type = visualization_msgs::msg::Marker::LINE_STRIP;
            point_vis_og.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point p1, p2;
            p1.x = path_matrix(0,0);
            p1.y = path_matrix(0,1);
            p1.z = path_matrix(0,2);

            p2.x = path_matrix(1,0);
            p2.y = path_matrix(1,1);
            p2.z = path_matrix(1,2);
            point_vis_og.points.push_back(p1);
            point_vis_og.points.push_back(p2);
            point_vis_og.scale.x = 0.05; // Line width
            point_vis_og.color.a = 0.8; // Transparency
            point_vis_og.color.r = 1.0; // Red
            point_vis_og.color.g = 0.64; // Green
            point_vis_og.color.b = 0.0; // Blue (for branches)
            path_visualizer.markers.push_back(point_vis_og);

        for(int i=1; i < path_matrix.rows(); i++)
        {
            visualization_msgs::msg::Marker point_vis;
            point_vis.header.frame_id = "ground_link";
            point_vis.header.stamp = this->get_clock()->now();
            point_vis.ns = "rrt_path";
            point_vis.id = marker_id++;
            point_vis.type = visualization_msgs::msg::Marker::LINE_STRIP;
            point_vis.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point p1, p2;
            p1.x = path_matrix(i-1,0);
            p1.y = path_matrix(i-1,1);
            p1.z = path_matrix(i-1,2);

            p2.x = path_matrix(i,0);
            p2.y = path_matrix(i,1);
            p2.z = path_matrix(i,2);
            point_vis.points.push_back(p1);
            point_vis.points.push_back(p2);
            point_vis.scale.x = 0.05; // Line width
            point_vis.color.a = 0.8; // Transparency
            point_vis.color.r = 1.0; // Red
            point_vis.color.g = 0.64; // Green
            point_vis.color.b = 0.0; // Blue (for branches)

            path_visualizer.markers.push_back(point_vis);
        }
        _vis_rrt_path_pub->publish(path_visualizer);
    }

    void visCommitTarget()
    {
        
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "ground_link";
            marker.header.stamp = this->now();
            marker.ns = "corridor";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set position of the marker
            marker.pose.position.x = _commit_target.x();
            marker.pose.position.y = _commit_target.y();
            marker.pose.position.z = _commit_target.z();

            // Set scale (diameter based on radius)
            double diameter = 2.0 * 0.25; // Radius to diameter
            marker.scale.x = diameter;
            marker.scale.y = diameter;
            marker.scale.z = diameter;

            // Set color and transparency
            marker.color.a = 0.5;  // Transparency
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            _vis_commit_target->publish(marker);
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


