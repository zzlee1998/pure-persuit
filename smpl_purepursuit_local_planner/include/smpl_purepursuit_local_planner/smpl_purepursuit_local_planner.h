//
// Created by zsy on 2022/09/06.
//


#ifndef SRC_smpl_purepursuit_local_planner_H
#define SRC_smpl_purepursuit_local_planner_H

#include <string>
#include <vector>

#include "costmap_2d/costmap_2d_ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_core/base_local_planner.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "smpl_msgs/FollowTrack.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

class PurePursuitPlanner : public nav_core::BaseLocalPlanner {
 public:
  PurePursuitPlanner();

  ~PurePursuitPlanner();

  void initialize(std::string name, tf2_ros::Buffer *tf,
				  costmap_2d::Costmap2DROS *costmap_ros) override;

  bool setPlan(
	  const std::vector<geometry_msgs::PoseStamped> &plan) override;

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;

  bool isGoalReached() override;

 private:
  double getEuclideanDistance(const geometry_msgs::PoseStamped &pose1,
							  const geometry_msgs::PoseStamped &pose2) const;

  void updateRobotPose();

  bool updateGoalReached(double &dist);

  bool transformPose(const std::string &frame,
					 const geometry_msgs::PoseStamped &in_pose,
					 geometry_msgs::PoseStamped &out_pose) const;

  bool transformPlan(
	  std::vector<geometry_msgs::PoseStamped> &transformed_plan);

  std::vector<geometry_msgs::PoseStamped> getTargetPoints(
	  const std::vector<geometry_msgs::PoseStamped> &plan, const double ahead_dist, const uint8_t size) const;

  nav_msgs::Path createPlanMsg(
	  const std::vector<geometry_msgs::PoseStamped> &plan) const;

  geometry_msgs::PointStamped createTargetMsg(
	  const geometry_msgs::PoseStamped &target) const;

  double computeLinearVel(double angular_error, double actual_dist, double turning_radius, double goal_dist);

  double
  computeSteeringAngle(double angular_error, double actual_dist, double turning_radius, double goal_dist) const;

  bool imminentCollision(const geometry_msgs::Twist &cmd_vel, bool &closer_to_collision);

  bool inCollision(const geometry_msgs::PoseStamped &pose) const;

  geometry_msgs::Pose2D calc_target_index(std::vector<geometry_msgs::PoseStamped> &poses, const geometry_msgs::Pose2D &robot, bool &end_is, int &robot_ind);
  geometry_msgs::Twist purePursuit_bypoint(const geometry_msgs::Pose2D &target, const geometry_msgs::Pose2D &robot, bool end_is);

  std::string name_;
  tf2_ros::Buffer *tf_;
  costmap_2d::Costmap2DROS *costmap_ros_;
  std::string map_frame_;
  double wheel_base_;
  bool cmd_angle_instead_rotvel_;
  double lookahead_dist_;
  int lookahead_poses_;
  double linear_vel_;
  double safe_linear_vel_;
  double min_scaling_vel_;
  double min_scaling_radius_;
  double decel_max_angular_error_;
  double max_angular_pos_;
  double transform_tolerance_;
  double goal_tolerance_;
  double collision_projected_time_;
  double decel_projected_time_;
  double dist_to_goal_;
  bool enable_vel_scaling_;
  bool enable_collision_checking_;
  ros::Publisher local_plan_pub_;
  ros::Publisher target_point_pub_;
  ros::Publisher collision_arc_pub_;
  bool is_plan_received_;
  bool is_goal_reached_;
  int angular_error_decel_times_;
  std::vector<geometry_msgs::PoseStamped> plan_;
  nav_msgs::Path local_plan_;
  geometry_msgs::PoseStamped robot_pose_;

  geometry_msgs::Twist base_vel_;
  double tolerance_trans_, tolerance_rot_;
  double ending_command_count_;
  double tolerance_timeout_, freq_;
  double max_vel_lin_, max_vel_th_;
  double min_vel_lin_, min_vel_th_;
  // double transform_tolerance_;
  double wheel_separation_h_;
  double forward_looking_distance_;
  double current_forward_factor_;
	bool extra_model_;

  double cur_tolerance_xy_;
  double cur_tolerance_theta_;
  double cur_looking_dis_;
  double cur_vel_max_;
  double cur_vel_min_;
  double cur_all_add_;
#define WORKMODE_SHORT (1) //短距离移动模式
#define WORKMODE_LONG (2) //长距离移动模式
  int curr_mode_;

  double front_width_, front_height_;
  double rear_width_, rear_height_;
  double safe_distance_;
};

#endif