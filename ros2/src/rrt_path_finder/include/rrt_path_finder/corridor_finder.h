#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Eigen>
#include <rrt_path_finder/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include "rrt_path_finder/datatype.h"

class safeRegionRrtStar
{
	private:
		pcl::search::KdTree<pcl::PointXYZ> kdtreeForMap;
		pcl::search::KdTree<pcl::PointXYZ> kdtreeAddMap;
		pcl::search::KdTree<pcl::PointXYZ> kdtreeDelMap;
		//pcl::PointCloud<pcl::PointXYZ> CloudIn;
		
		kdtree * kdTree_; // dynamic light-weight Kd-tree, for organizing the nodes in the exploring tree

		vector<int>     pointIdxRadiusSearch;
		vector<float>   pointRadiusSquaredDistance;        
		
		// All nodes,  nodes reach the target
		vector<NodePtr> NodeList, EndList;
		
		// all nodes in the current path, for easy usage in root re-decleration 
		vector<NodePtr> PathList, invalidSet;
		
		// record the current best node which can reach the target and has the lowest path cost
		NodePtr best_end_ptr;

		// record the root of the rapidly exploring tree
		NodePtr root_node;

		// start point,   target point,  centroid point of the ellipsoide sampling region
		Eigen::Vector3d start_pt, end_pt, inform_centroid, commit_root;

		// ctrl the size of the trash nodes cach, once it's full, remove them and rebuild the kd-tree
		int cach_size = 100; 

		// maximum allowed samples
		int max_samples = 30000;

		// used for basic of sampling-based path finding
		double x_l, x_h, y_l, y_h, z_l, z_h, bias_l, bias_h, inlier_ratio, goal_ratio;
		
		// used for range query and search strategy
		double safety_margin, max_radius, search_margin, sample_range;
		
		// used for the informed sampling strategy
		double min_distance, best_distance, elli_l, elli_s, ctheta, stheta;   
		
		// FLAGs
		bool inform_status, path_exist_status, global_navi_status;

		Eigen::MatrixXd Path;
		Eigen::VectorXd Radius;
		
		random_device rd;
		default_random_engine eng;
		uniform_real_distribution<double>  rand_rho = uniform_real_distribution<double>(0.0, 1.0);  // random distribution for generating samples inside a unit circle
		uniform_real_distribution<double>  rand_phi = uniform_real_distribution<double>(0.0, 2 * M_PI);
		uniform_real_distribution<double>  rand_x,    rand_y,    rand_z,   rand_bias; // basic random distributions for generating samples, in all feasible regions
		uniform_real_distribution<double>  rand_x_in, rand_y_in, rand_z_in; // random distribution, especially for generating samples inside the local map's boundary

    /* ---------- uniformly sampling a 3D ellipsoid ---------- */
    uniform_real_distribution<double> rand_u, rand_v;  // U ~ (0,1)
    Eigen::Vector3d translation_inf;
    Eigen::Matrix3d rotation_inf;

  public:
		safeRegionRrtStar( );
        ~safeRegionRrtStar();
		
		/* set-up functions */
		void reset();
		void setParam( double safety_margin_, double search_margin_, double max_radius_, double sample_range_ );
		void setInput(pcl::PointCloud<pcl::PointXYZ> CloudIn);
		void setPt( Eigen::Vector3d startPt, Eigen::Vector3d endPt, double xl, double xh, double yl, double yh, double zl, double zh,
					double local_range, int max_iter, double sample_portion, double goal_portion );
		void setStartPt( Eigen::Vector3d startPt, Eigen::Vector3d endPt);

		/*  commit local target and move tree root  */
		void resetRoot(Eigen::Vector3d & commitTarget);
		void resetRoot(float sensing_range);
		void solutionUpdate(double cost_reduction, Eigen::Vector3d target);

		/* main function entries */
		void SafeRegionExpansion( double time_limit);
		void SafeRegionRefine   ( double time_limit);
		void SafeRegionEvaluate ( double time_limit);

		/* operations on the tree */
		void treeRepair   ( double time_limit, vector< pair<Eigen::Vector3d, double> > & node_list);
		void treePrune( NodePtr newPtr );
		void treeRewire( NodePtr node_new_ptr, NodePtr node_nearst_ptr );
		void clearBranchW(NodePtr node_delete); // weak branch cut:    clear branches while avoid deleting the nodes on the best path
		void clearBranchS(NodePtr node_delete); // strong branch cut:  clear all nodes on a branch
		void updateHeuristicRegion( NodePtr update_end_node );
		void recordNode(NodePtr new_node);		
		void removeInvalid();
		void treeDestruct();
		void tracePath();

		/* utility functions */
        inline double getDis(const NodePtr node1, const NodePtr node2);
		inline double getDis(const NodePtr node1, const Eigen::Vector3d & pt);
		inline double getDis(const Eigen::Vector3d & p1, const Eigen::Vector3d & p2);
		inline Eigen::Vector3d genSample();
		inline double radiusSearch(Eigen::Vector3d & pt);
		inline NodePtr genNewNode( Eigen::Vector3d & pt, NodePtr node_nearst_ptr );
		inline NodePtr findNearstVertex( Eigen::Vector3d & pt );
		inline NodePtr findNearst( Eigen::Vector3d & pt );
		inline double checkRadius(Eigen::Vector3d & pt);
		inline bool checkValidEnd(NodePtr endPtr);
		inline bool checkEnd( NodePtr ptr );
		inline int  checkNodeUpdate  ( double new_radius, double old_radius);
		inline int  checkNodeRelation( double dis, NodePtr node_1, NodePtr node_2 );
		inline bool isSuccessor(NodePtr curPtr, NodePtr nearPtr);
		bool checkTrajPtCol(Eigen::Vector3d & pt);

		/* data return */
		pair<Eigen::MatrixXd, Eigen::VectorXd> getPath()
		{
			return make_pair(Path, Radius);
		};
		
		vector<NodePtr> getTree()
		{
			return NodeList;
		};

		bool getPathExistStatus()
		{
			return path_exist_status;
		};

		bool getGlobalNaviStatus()
		{
			return global_navi_status;
		};
};