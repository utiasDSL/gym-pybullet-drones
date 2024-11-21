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

#include "rrt_path_finder/corridor_finder.h"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

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

        this->declare_parameter("safety_margin", 0.75);
        this->declare_parameter("search_margin", 0.50);
        this->declare_parameter("max_radius", 1.5);
        this->declare_parameter("sensing_range", 2.0);
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
        _vis_rrt_star_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("_vis_rrt_tree", 1);
        _vis_corridor_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("viscorridor", 1);
        _vis_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("_vis_pcd", 1);

        // Add the RRT waypoints publisher
        _rrt_waypoints_pub = this->create_publisher<nav_msgs::msg::Path>("rrt_waypoints", 1);

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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _vis_rrt_star_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _vis_corridor_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _vis_map_pub;
    // RRT waypoints publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _rrt_waypoints_pub;

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
    double _planning_rate, _safety_margin, _search_margin, _max_radius, _sensing_range, _replan_distance;
    double _refine_portion, _sample_portion, _goal_portion, _path_find_limit, _stop_time, _time_commit;
    double _x_l, _x_h, _y_l, _y_h, _z_l, _z_h;  // For random map simulation: map boundary

    int _max_samples;

    // RRT Path Planner
    safeRegionRrtStar _rrtPathPlanner;

    // Variables for target position, trajectory, odometry, etc.
    Eigen::Vector3d _start_pos, _end_pos, _start_vel, _start_acc, _commit_target;
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
        _vis_rrt_star_pub->publish(tree_markers);

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



    // Function to plan the initial trajectory using RRT
    void planInitialTraj()
    {
        _rrtPathPlanner.reset();

        // Set parameters for the RRT planner
        // safety_margin=1.0, search_margin=0.5, max_radius=1.5, sample_range=10.0

        _rrtPathPlanner.setPt(_start_pos, _end_pos, _x_l, _x_h, _y_l, _y_h, _z_l, _z_h,
                             _max_radius, _max_samples, _stop_time, _time_commit);

        _rrtPathPlanner.SafeRegionExpansion(_path_find_limit);
        auto [path, radius] = _rrtPathPlanner.getPath();

        if (_rrtPathPlanner.getPathExistStatus())
        {
            // Publish the RRT waypoints
            std::vector<Eigen::Vector3d> path_vector = matrixToVector(path);
            std::vector<double> radius_vector = radiusMatrixToVector(radius);
            publishRRTWaypoints(path_vector);
            publishCorridorVisualization(path_vector, radius_vector);

            // Temporary setting root based on distance
            // _rrtPathPlanner.resetRoot(_replan_distance);

            // Update trajectory existence status
            _is_traj_exist = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No path found in initial trajectory planning");
        }
        visRrt(_rrtPathPlanner.getTree()); 
    }

    // Function to plan the incremental trajectory
    void planIncrementalTraj()
    {
        if (_rrtPathPlanner.getGlobalNaviStatus())
        {
            visRrt(_rrtPathPlanner.getTree()); 
            return; // No further planning required if global navigation is complete
        }

        if (_is_target_arrive)
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
                // _rrtPathPlanner.resetRoot(_replan_distance);
                RCLCPP_WARN(this->get_logger(), "Trajectory replan triggered");
                // Get the updated path and publish it
                auto [path, radius] = _rrtPathPlanner.getPath();
                std::vector<Eigen::Vector3d> path_vector = matrixToVector(path);
                std::vector<double> radius_vector = radiusMatrixToVector(radius);
                publishRRTWaypoints(path_vector);
                publishCorridorVisualization(path_vector, radius_vector);

                // Further processing or visualization here
            }
        }
        else
        {
            // Continue refining and evaluating the path
            _rrtPathPlanner.SafeRegionRefine(_refine_portion);
            _rrtPathPlanner.SafeRegionEvaluate(_goal_portion);

            // Get the updated path and publish it
            auto [path, radius] = _rrtPathPlanner.getPath();
            std::vector<Eigen::Vector3d> path_vector = matrixToVector(path);
            std::vector<double> radius_vector = radiusMatrixToVector(radius);
            publishRRTWaypoints(path_vector);
            publishCorridorVisualization(path_vector, radius_vector);
            
        }
        //RCLCPP_DEBUG(this->get_logger(),"Traj updated");
        visRrt(_rrtPathPlanner.getTree()); 

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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPlanner>());
    rclcpp::shutdown();
    return 0;
}
