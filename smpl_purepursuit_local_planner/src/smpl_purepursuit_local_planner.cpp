//
// Created by zsy on 2022/09/06.
//

#include <algorithm>
#include <cmath>
#include <iterator>

#include "costmap_2d/costmap_2d.h"
#include "geometry_msgs/Quaternion.h"
#include "pluginlib/class_list_macros.h"
#include "ros/ros.h"
#include "tf2/utils.h"

#include "smpl_purepursuit_local_planner/smpl_purepursuit_local_planner.h"

PLUGINLIB_EXPORT_CLASS(PurePursuitPlanner,
	nav_core::BaseLocalPlanner
)

PurePursuitPlanner::PurePursuitPlanner() {}

PurePursuitPlanner::~PurePursuitPlanner() {}

void PurePursuitPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
									costmap_2d::Costmap2DROS *costmap_ros) {
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("wheel_base", wheel_base_, 0.3);
  private_nh.param<bool>("cmd_angle_instead_rotvel", cmd_angle_instead_rotvel_, false);
  private_nh.param("lookahead_dist", lookahead_dist_, 0.5);
  private_nh.param("lookahead_poses", lookahead_poses_, 3);
  private_nh.param("linear_vel", linear_vel_, 0.5);
  private_nh.param("safe_linear_vel", safe_linear_vel_, 0.2);
  private_nh.param("min_scaling_vel", min_scaling_vel_, 0.1);
  private_nh.param("decel_max_angular_error", decel_max_angular_error_, 0.3);
  private_nh.param("min_scaling_radius", min_scaling_radius_, 0.5);
  private_nh.param("max_angular_pos", max_angular_pos_, 0.5);
  private_nh.param("transform_tolerance", transform_tolerance_, 0.2);
  private_nh.param("goal_tolerance", goal_tolerance_, 0.2);

  private_nh.param("decel_projected_time", decel_projected_time_, 4.0);
  private_nh.param("collision_projected_time", collision_projected_time_, 0.4);
  private_nh.param("enable_vel_scaling", enable_vel_scaling_, false);
  private_nh.param("enable_collision_checking", enable_collision_checking_, false);

  is_plan_received_ = false;
  is_goal_reached_ = false;
  dist_to_goal_ = 0;
  angular_error_decel_times_ = 0;

  local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
  target_point_pub_ = private_nh.advertise<smpl_msgs::FollowTrack>("target_point", 1);
  collision_arc_pub_ = private_nh.advertise<nav_msgs::Path>("collision_arc", 1);

  ROS_INFO("PurePursuitPlanner initialized");
}

bool PurePursuitPlanner::setPlan(
	const std::vector<geometry_msgs::PoseStamped> &plan) {

  // local_plan_.header = msg->header;
  // local_plan_.poses.resize = plan.size();
  local_plan_.poses = plan;

  plan_ = plan;
  const bool is_plan_empty = plan_.empty();
  if (!is_plan_empty) {
    ROS_INFO("PurePursuitPlanner: New plan Received");
    is_plan_received_ = true;
  } else {
    ROS_ERROR("PurePursuitPlanner: Cannot set new plan");
    is_plan_received_ = false;
  }
  is_goal_reached_ = false;
  dist_to_goal_ = 0.0;
  return !is_plan_empty;
}
#if 0
bool PurePursuitPlanner::computeVelocityCommands(
	geometry_msgs::Twist &cmd_vel) {
  updateRobotPose();
  double goal_dist = 0;
  if (!updateGoalReached(goal_dist)) {
    ROS_ERROR("PurePursuitPlanner: Cannot update whether goal is reached");
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> plan;
  std::vector<geometry_msgs::PoseStamped> target_points;
  geometry_msgs::PoseStamped target_point;
  double angular_error, actual_dist;
  if (transformPlan(plan)) {
    target_points = getTargetPoints(plan, lookahead_dist_, lookahead_poses_);
    target_point = target_points.at(0);
    angular_error = std::atan2(target_point.pose.position.y, target_point.pose.position.x);
    actual_dist = std::hypot(target_point.pose.position.x,
                target_point.pose.position.y);

    local_plan_pub_.publish(createPlanMsg(plan));
  } else {
    ROS_ERROR("PurePursuitPlanner: Cannot transform plan");
    return false;
  }
  const double curvature = std::fabs(2*std::sin(angular_error)/actual_dist);
  const double turning_radius = 1/curvature;
  if (is_plan_received_ && !is_goal_reached_) {
    if (enable_vel_scaling_) {
      cmd_vel.linear.x = computeLinearVel(angular_error, actual_dist, turning_radius, goal_dist);
    } else {
      cmd_vel.linear.x = linear_vel_;
    }
    cmd_vel.angular.z = computeSteeringAngle(angular_error, actual_dist, turning_radius, goal_dist);
  } else {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
  }
  cmd_vel.angular.z = std::max(-1*std::abs(max_angular_pos_),
							   std::min(std::abs(max_angular_pos_), cmd_vel.angular.z));

  smpl_msgs::FollowTrack ft;
  ft.header.stamp = ros::Time::now();
  ft.goal_dist = goal_dist;
  ft.angular_error = angular_error;
  ft.actual_dist = actual_dist;
  ft.curvature = curvature;
  ft.turning_radius = turning_radius;
  ft.ahead_poses.resize(0);
  geometry_msgs::PoseStamped pose;
  for (int i = 0; i < target_points.size(); i++) {
    if (!transformPose(costmap_ros_->getGlobalFrameID(), target_points[i], pose)) {
      ROS_ERROR("PurePursuitPlanner: Cannot transform pose");
    } else {
      ft.ahead_poses.push_back(pose);
    }
  }
  ft.twist = cmd_vel;
  target_point_pub_.publish(ft);
  if (enable_collision_checking_) {
    bool closer_to_collision = false;
    bool imminent_collision = imminentCollision(cmd_vel, closer_to_collision);
	if (imminent_collision) {
	  return false;
	}
	if (closer_to_collision) {
	  cmd_vel.linear.x = safe_linear_vel_;
	}
  }
  return true;
}
#else
bool PurePursuitPlanner::computeVelocityCommands(
	geometry_msgs::Twist &cmd_vel) {

  updateRobotPose();
  double goal_dist = 0;
  if(!updateGoalReached(goal_dist)) {
    ROS_ERROR("PurePursuitPlanner: Cannot update whether goal is reached");
    return false;
  }

  std::vector<geometry_msgs::PoseStamped> plan =plan_;
  std::vector<geometry_msgs::PoseStamped> target_points;
  geometry_msgs::PoseStamped target_point;
  double angular_error, actual_dist;
  nav_msgs::Path local_plan;
  // if (transformPlan(plan)) {
  //   target_points = getTargetPoints(plan, lookahead_dist_, lookahead_poses_);
  //   target_point = target_points.at(0);
  // } else {
  //   ROS_ERROR("PurePursuitPlanner: Cannot transform plan");
  //   return false;
  // }
  if(plan.size()==0) {
    ROS_ERROR("PurePursuitPlanner: Cannot transform plan");
    return false;
  }

  curr_mode_ = WORKMODE_LONG;
  if(curr_mode_ == WORKMODE_SHORT) {
    cur_tolerance_xy_ = tolerance_trans_;
    cur_tolerance_theta_ = tolerance_rot_;
    cur_looking_dis_ = forward_looking_distance_*3.0f;
    cur_vel_max_ = max_vel_lin_*1.5f;
    cur_vel_min_= min_vel_lin_;
    cur_all_add_ = 3.0f;
}else if(curr_mode_ == WORKMODE_LONG){
    cur_tolerance_xy_ = tolerance_trans_ * 5.0f;
    cur_tolerance_theta_ = tolerance_rot_ * 5.0f;
    cur_looking_dis_ = forward_looking_distance_*5.0f;
    cur_vel_max_ = max_vel_lin_ * 4.0f;
    cur_vel_min_= min_vel_lin_ * 4.0f;
    cur_all_add_ = 5.0f;
}

  geometry_msgs::Pose2D robot_pose;
  robot_pose.x = robot_pose_.pose.position.x;
  robot_pose.y = robot_pose_.pose.position.y;
  robot_pose.theta = tf::getYaw(robot_pose_.pose.orientation);
  geometry_msgs::Pose2D target_pose;
  bool end_is=false;
  int robot_ind=0;
  target_pose = calc_target_index(plan, robot_pose, end_is, robot_ind);
  // geometry_msgs::Twist cmd_vel = purePursuit_bypoint(target_pose, robot_pose, end_is);

}

geometry_msgs::Pose2D PurePursuitPlanner::calc_target_index(std::vector<geometry_msgs::PoseStamped> &poses, const geometry_msgs::Pose2D &robot, bool &end_is, int &robot_ind)
{
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Pose2D pose_index;
    // vector<double> distanceRobot;
    double min_distance=9999;
    double dis=0, check_dis=0;
    int min_ind = 0, forward_ind = 0;
    // double Lf = k * v + Lfc;
    double Lf = cur_looking_dis_ + cur_vel_max_, L = 0;

    for(int ind = robot_ind; ind < poses.size() && check_dis<Lf; ind++) {//
        double x = poses[ind].pose.position.x;
        double y = poses[ind].pose.position.y;
        double min_theta = tf::getYaw(poses[ind].pose.orientation);
        min_theta = min_theta>=0?min_theta:(2*M_PI + min_theta);
        double robot_theta = robot.theta>=0?robot.theta:(2*M_PI+robot.theta);
        // if(fabs(min_theta-robot_theta)>M_PI/6.0f) {// 剔除与当前方向差距过大的点
        //     continue;
        // }
        if(ind>robot_ind) {
            check_dis += sqrt(pow(poses[ind].pose.position.x - poses[ind-1].pose.position.x, 2) + pow(poses[ind].pose.position.y - poses[ind-1].pose.position.y, 2));
        }
        dis = sqrt(pow(x-robot.x, 2)+pow(y-robot.y, 2));
        if(min_distance > dis) {
            min_distance = dis;
            min_ind = ind;
        }
        // distanceRobot.push_back(dis);
    }
    robot_ind = min_ind;

    forward_ind = min_ind;
    double min_theta=tf::getYaw(poses[min_ind].pose.orientation);
    // min_theta = fabs(min_theta)>=0?min_theta:(2*M_PI + min_theta);
    double forward_theta=tf::getYaw(poses[forward_ind].pose.orientation);
    // forward_theta = forward_theta>=0?forward_theta:(2*M_PI + forward_theta);
    double sub_theta = fabs(min_theta-forward_theta);
    if(sub_theta>M_PI) {
        sub_theta = sub_theta - 2*M_PI;
    }
    double min_x = (poses[forward_ind+1].pose.position.x-poses[forward_ind].pose.position.x) * cos(forward_theta) + \
            (poses[forward_ind+1].pose.position.y-poses[forward_ind].pose.position.y) * sin(forward_theta);
    bool rev = true;
    while(Lf>min_distance && Lf >L && \
        (forward_ind+1)<poses.size() && \
        fabs(sub_theta)<M_PI/9.0f) {
        double x = poses[forward_ind].pose.position.x;
        double y = poses[forward_ind].pose.position.y;
        double x_1 = poses[forward_ind+1].pose.position.x;
        double y_1 = poses[forward_ind+1].pose.position.y;
        dis = sqrt(pow(x_1 - x, 2) + pow(y_1 - y, 2));
        L += dis;
        forward_ind++;
        forward_theta=tf::getYaw(poses[forward_ind].pose.orientation);
        if(rev) {
            double forward_x = (poses[forward_ind+1].pose.position.x-poses[forward_ind].pose.position.x) * cos(forward_theta) + \
                (poses[forward_ind+1].pose.position.y-poses[forward_ind].pose.position.y) * sin(forward_theta);
            if((min_x>0&&forward_x<0) || (min_x<0&&forward_x>0)) {
                if(L>cur_vel_min_) {
                    break;
                }else {
                    rev = false;
                }
            }
        }
        forward_theta = forward_theta>=0?forward_theta:(2*M_PI + forward_theta);
        sub_theta = fabs(min_theta-forward_theta);
        if(sub_theta>M_PI) {
            sub_theta = sub_theta - 2*M_PI;
        }
    }

    pose_index.x = poses[forward_ind].pose.position.x;
    pose_index.y = poses[forward_ind].pose.position.y;
    pose_index.theta = tf::getYaw(poses[forward_ind].pose.orientation);
    if(forward_ind >= poses.size()-1) {
        end_is = true;
    }else {
        end_is = false;
    }

    return pose_index;
}

geometry_msgs::Twist PurePursuitPlanner::purePursuit_bypoint(const geometry_msgs::Pose2D &target, const geometry_msgs::Pose2D &robot, bool end_is)
{
    geometry_msgs::Twist res;
    bool only_rot=false;

    if(end_is && fabs(target.y - robot.y)<cur_tolerance_xy_ && fabs(target.x - robot.x)<cur_tolerance_xy_ ) {
        only_rot = true;
        double offset_theta;
        offset_theta = target.theta - robot.theta;
        if(fabs(offset_theta)>M_PI) {
            offset_theta = offset_theta - (offset_theta>0?2*M_PI:(-2*M_PI));
        }
        if(fabs(offset_theta)<cur_tolerance_theta_) {
            ROS_INFO("[%s] goal reach.", name_.c_str());
            res.linear.x = 0;
            res.angular.z = 0;
            is_goal_reached_ = true;
            return res;
        }
    }

    double y = (target.y-robot.y) * cos(robot.theta) - (target.x-robot.x) * sin(robot.theta);
    double x = (target.x-robot.x) * cos(robot.theta) + (target.y-robot.y) * sin(robot.theta);

    double theta_t = target.theta>=0?target.theta:(2*M_PI + target.theta);
    double theta_r = robot.theta>=0?robot.theta:(2*M_PI + robot.theta);
    double vel_x = 0.0;
    double angle_z_mul = 1.0f;
    if(end_is) {
        vel_x = cur_vel_min_;
    }else {
        vel_x = cur_vel_max_;
    }
    if(fabs(theta_t-theta_r)>M_PI/9.0f) {//转角较大时，使用最小速度
        if(curr_mode_==WORKMODE_LONG) {
            vel_x = cur_vel_min_;
        }
    }
    if(curr_mode_==WORKMODE_LONG) {
        angle_z_mul = 0.95;//角度补偿，降低角度控制慢于速度控制的影响
    }
    if(x<0) {
        res.linear.x = -vel_x;
    }else {
        res.linear.x = vel_x;
    }

    double alpha;
    if(end_is) { // && only_rot 
        alpha = target.theta - robot.theta;
    }else {
        alpha = atan2(target.y - robot.y, target.x - robot.x) - robot.theta;
    }
    if(x<0 && end_is) {// && only_rot
        alpha = alpha + M_PI;
    }

    double Ld = sqrt((target.y - robot.y) * (target.y - robot.y)+  (target.x - robot.x)* (target.x - robot.x));
    double Lf = fabs(res.linear.x) + cur_looking_dis_;
    double delta;
    delta = atan(2.0 * wheel_separation_h_ * sin(alpha) / Ld);
    if(!end_is) {
    //     delta = atan(2.0 * wheel_separation_h_ * sin(alpha) / Lf);
        delta /= angle_z_mul;

    }

    if(fabs(delta) > M_PI/3.0f) {
        res.angular.z = M_PI/3.0f * (delta > 0 ?1:(-1));
    }else {
        res.angular.z = delta;
    }

    // double max_vel = Ld * freq_ / K_rot_ * cos(res.angular.z); // v = Ld / dt
    double max_vel = Ld/2.0f;
    // ROS_INFO("[%s] max_vel:%f.", name_.c_str(), max_vel);
    res.linear.x = std::max(std::min(res.linear.x, max_vel), -max_vel);
    if(res.linear.x<0) {
        res.linear.x = std::min(res.linear.x, -cur_vel_min_);
    }else {
        res.linear.x = std::max(res.linear.x, cur_vel_min_);
    }

    if(end_is && fabs(res.angular.z-base_vel_.angular.z)>M_PI/120.0f) {//
        res.linear.x = 0;
    }else if(fabs(res.angular.z-base_vel_.angular.z)>M_PI/90.0f*cur_all_add_) {//
        res.linear.x = 0;
    }
    // checkObstacleIs(res.linear.x, robot);
    // ROS_INFO("[%s] purePursuit_bypoint x:%f , z:%f , delta:%f , alpha:%f", name_.c_str(),  res.linear.x, res.angular.z, delta, alpha);

    return res;
}

#if 0
bool MovePurePursuit::checkObstacle_area_is_exist(std::string pre_name, const geometry_msgs::Pose2D &robot, int &obstacle_exist_count, \
    const int ob_min_w, const int ob_max_w, const int ob_min_h, const int ob_max_h)
{
    int min_w=9999;
    int max_w=0;
    int min_h=9999;
    int max_h=0;
    // ROS_INFO("name(%s), w:%d~%d , h:%d~%d.", pre_name.c_str(), ob_min_w, ob_max_w, ob_min_h, ob_max_h);

    double resolution = local_costmap_.info.resolution;
    geometry_msgs::Pose origin = local_costmap_.info.origin;
    int robot_w = (robot.x-origin.position.x)/resolution;
    int robot_h = (robot.y-origin.position.y)/resolution;

    int costmap_width = local_costmap_.info.width;
    int costmap_height = local_costmap_.info.height;
    double robot_theta = -robot.theta;
    // int ob_min_w = (front_width);
    // int ob_max_w = (front_width+front_safe_distance);
    // int ob_min_h = (-front_height);
    // int ob_max_h = (front_height);

#if 1
    tf::Transform transform;
    transform.setRotation(tf::Quaternion(origin.orientation.x, origin.orientation.y, \
        origin.orientation.z, origin.orientation.w));
    // ob_min_w+ob_max_h
    transform.setOrigin(tf::Vector3(origin.position.x+(ob_min_w*cos(robot_theta)+ob_max_h*sin(robot_theta)+robot_w)*resolution, \
        origin.position.y+(ob_max_h*cos(robot_theta)-ob_min_w*sin(robot_theta)+robot_h)*resolution, origin.position.z));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", pre_name+"ob_1"));
    // ob_min_w+ob_min_h
    transform.setOrigin(tf::Vector3(origin.position.x+(ob_min_w*cos(robot_theta)+ob_min_h*sin(robot_theta)+robot_w)*resolution, \
        origin.position.y+(ob_min_h*cos(robot_theta)-ob_min_w*sin(robot_theta)+robot_h)*resolution, origin.position.z));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", pre_name+"ob_2"));
    // ob_max_w+ob_max_h
    transform.setOrigin(tf::Vector3(origin.position.x+(ob_max_w*cos(robot_theta)+ob_max_h*sin(robot_theta)+robot_w)*resolution, \
        origin.position.y+(ob_max_h*cos(robot_theta)-ob_max_w*sin(robot_theta)+robot_h)*resolution, origin.position.z));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", pre_name+"ob_3"));
    // ob_max_w+ob_min_h
    transform.setOrigin(tf::Vector3(origin.position.x+(ob_max_w*cos(robot_theta)+ob_min_h*sin(robot_theta)+robot_w)*resolution, \
        origin.position.y+(ob_min_h*cos(robot_theta)-ob_max_w*sin(robot_theta)+robot_h)*resolution, origin.position.z));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", pre_name+"ob_4"));
#endif

    min_w = std::min((double)min_w, ob_min_w*cos(robot_theta)+ob_max_h*sin(robot_theta));
    min_w = std::min((double)min_w, ob_min_w*cos(robot_theta)+ob_min_h*sin(robot_theta));
    min_w = std::min((double)min_w, ob_max_w*cos(robot_theta)+ob_max_h*sin(robot_theta));
    min_w = std::min((double)min_w, ob_max_w*cos(robot_theta)+ob_min_h*sin(robot_theta));
    min_w = std::max(min_w+robot_w, 0);
    max_w = std::max((double)max_w, ob_min_w*cos(robot_theta)+ob_max_h*sin(robot_theta));
    max_w = std::max((double)max_w, ob_min_w*cos(robot_theta)+ob_min_h*sin(robot_theta));
    max_w = std::max((double)max_w, ob_max_w*cos(robot_theta)+ob_max_h*sin(robot_theta));
    max_w = std::max((double)max_w, ob_max_w*cos(robot_theta)+ob_min_h*sin(robot_theta));
    max_w = std::min(max_w+robot_w, costmap_width);

    min_h = std::min((double)min_h, ob_min_h*cos(robot_theta)-ob_max_w*sin(robot_theta));
    min_h = std::min((double)min_h, ob_min_h*cos(robot_theta)-ob_min_w*sin(robot_theta));
    min_h = std::min((double)min_h, ob_max_h*cos(robot_theta)-ob_max_w*sin(robot_theta));
    min_h = std::min((double)min_h, ob_max_h*cos(robot_theta)-ob_min_w*sin(robot_theta));
    min_h = std::max(min_h+robot_h, 0);
    max_h = std::max((double)max_h, ob_min_h*cos(robot_theta)-ob_max_w*sin(robot_theta));
    max_h = std::max((double)max_h, ob_min_h*cos(robot_theta)-ob_min_w*sin(robot_theta));
    max_h = std::max((double)max_h, ob_max_h*cos(robot_theta)-ob_max_w*sin(robot_theta));
    max_h = std::max((double)max_h, ob_max_h*cos(robot_theta)-ob_min_w*sin(robot_theta));
    max_h = std::min(max_h+robot_h, costmap_height);

    // obstacle_count = 0;
    // ROS_INFO("robot_theta(%f) robot_w:%d(%d) robot_h:%d(%d)", robot_theta, robot_w, costmap_width/2, robot_h, costmap_height/2);
    // ROS_INFO("[%s]w(%d - %d) h(%d - %d)", pre_name.c_str(), min_w, max_w, min_h, max_h);
    for(int w=min_w; w < max_w; ++w) {
        for(int h=min_h; h < max_h; ++h) {
            int w_cal = w-robot_w;
            int h_cal = h-robot_h;
            int w_theta = w_cal*cos(-robot_theta)+h_cal*sin(-robot_theta);
            int h_theta = h_cal*cos(-robot_theta)-w_cal*sin(-robot_theta);
            if(w_theta>ob_min_w && w_theta<ob_max_w && h_theta>ob_min_h && h_theta<ob_max_h) {
                if(local_costmap_.data[h*costmap_width+w]==100) {
                    obstacle_exist_count++;
                }
            }
        }
    }

    if(obstacle_exist_count>0) {
        ROS_WARN("exist obstacle_count:%d.", obstacle_exist_count);
        // linear_vel = 0;
        return true;
    }else {
        return false;
    }
}

#define OB_COUNT_STD (3)
void MovePurePursuit::checkObstacleIs(double &linear_vel, const geometry_msgs::Pose2D &robot)
{
    if(linear_vel==0) {
        return ;
    }

    // linear_vel = 0.2;
    // curr_mode_=WORKMODE_LONG;
    double front_safe_distance_ = safe_distance_+linear_vel*2.0f;
    double rear_safe_distance_ = -safe_distance_-linear_vel*2.0f;

    int obstacle_count = 0;
    double robot_theta = -robot.theta;

    if(local_costmap_.info.width==0 || local_costmap_.info.height==0 ||  local_costmap_.data.size()==0) {
        ROS_INFO("no local_costmap_ data.");
        ROS_INFO("local_costmap_ data(width:%d, height:%d data_size:%d).", local_costmap_.info.width, local_costmap_.info.height, local_costmap_.data.size());
        return ;
    }
    double resolution = local_costmap_.info.resolution;
    int front_width = front_width_/resolution;
    int front_height = 0;
    if(extra_model_) {
        front_height = (front_height_+0.3)/resolution;
    }else {
        front_height = front_height_/resolution;
    }
    int front_safe_distance = front_safe_distance_/resolution;
    int rear_width = rear_width_/resolution;
    int rear_height = rear_height_/resolution;
    int rear_safe_distance = rear_safe_distance_/resolution;

    if(linear_vel>0 && curr_mode_==WORKMODE_LONG) {
        int ob_min_w = (front_width);
        int ob_max_w = (front_width+front_safe_distance);
        int ob_min_h = (-front_height);
        int ob_max_h = (front_height);
        obstacle_count = 0;
        checkObstacle_area_is_exist("front_extra_", robot, obstacle_count, ob_min_w, ob_max_w, ob_min_h, ob_max_h);

        if(obstacle_count>OB_COUNT_STD) {
            ROS_WARN("front obstacle_count:%d.", obstacle_count);
            linear_vel = 0;
        }
    }else if(linear_vel>0 && curr_mode_==WORKMODE_SHORT) {
        int ob_min_w = (front_width);
        int ob_max_w = (front_width+front_safe_distance);
        int ob_min_h = (-front_height);
        int ob_max_h = (front_height);
        obstacle_count = 0;
        checkObstacle_area_is_exist("front_", robot, obstacle_count, ob_min_w, ob_max_w, ob_min_h, ob_max_h);

        if(obstacle_count>OB_COUNT_STD) {
            ROS_WARN("front obstacle_count:%d.", obstacle_count);
            linear_vel = 0;
        }
    }
    // linear_vel = -0.2;
    if(linear_vel<0 && curr_mode_==WORKMODE_LONG) {
        int ob_min_w = (rear_width+rear_safe_distance);
        int ob_max_w = (rear_width);
        int ob_min_h = (-rear_height);
        int ob_max_h = (rear_height);
        obstacle_count = 0;
        checkObstacle_area_is_exist("rear_", robot, obstacle_count, ob_min_w, ob_max_w, ob_min_h, ob_max_h);

        if(obstacle_count>OB_COUNT_STD) {
            ROS_WARN("rear obstacle_count:%d.", obstacle_count);
            linear_vel = 0;
        }
    }else if(linear_vel<0 && curr_mode_==WORKMODE_SHORT) { // LRF cha che moxing
        int ob_min_w = (rear_width+rear_safe_distance);
        int ob_max_w = (rear_width);
        int ob_min_h = (-rear_height);
        int ob_max_h = (-rear_height+0.12/resolution);
        obstacle_count = 0;
        checkObstacle_area_is_exist("rear1_", robot, obstacle_count, ob_min_w, ob_max_w, ob_min_h, ob_max_h);

        if(obstacle_count>OB_COUNT_STD) {
            ROS_WARN("rear1 obstacle_count:%d.", obstacle_count);
            linear_vel = 0;
        }

        ob_min_h = (rear_height-0.12/resolution);
        ob_max_h = (rear_height);
        obstacle_count = 0;
        checkObstacle_area_is_exist("rear2_", robot, obstacle_count, ob_min_w, ob_max_w, ob_min_h, ob_max_h);

        if(obstacle_count>OB_COUNT_STD) {
            ROS_WARN("rear2 obstacle_count:%d.", obstacle_count);
            linear_vel = 0;
        }
    }
}
#endif

#endif

bool PurePursuitPlanner::isGoalReached() {
  if (is_goal_reached_) {
	  ROS_INFO("PurePursuitPlanner: Goal reached");
  }
  return is_goal_reached_;
}

double PurePursuitPlanner::getEuclideanDistance(
	const geometry_msgs::PoseStamped &pose1,
	const geometry_msgs::PoseStamped &pose2) const {
  const double dx = pose1.pose.position.x - pose2.pose.position.x;
  const double dy = pose1.pose.position.y - pose2.pose.position.y;
  return std::hypot(dx, dy);
}

void PurePursuitPlanner::updateRobotPose() {
  costmap_ros_->getRobotPose(robot_pose_);
}

bool PurePursuitPlanner::updateGoalReached(double &dist) {
  geometry_msgs::PoseStamped robot_pose;

  if (!transformPose(plan_[0].header.frame_id, robot_pose_, robot_pose)) {
    ROS_ERROR("PurePursuitPlanner: Cannot transform robot pose");
    return false;
  }

  const double distance_to_goal = getEuclideanDistance(plan_.back(), robot_pose);
  dist = distance_to_goal;
  ROS_DEBUG_STREAM("distance_to_goal: " << distance_to_goal << " distance_to_goal:" << dist_to_goal_);

  // if (dist < std::max(goal_tolerance_*2, 0.5) && dist_to_goal_ <= 0) {
  //   dist_to_goal_ = distance_to_goal;
  // }
  // if (dist_to_goal_ > 0) {
  //   if (distance_to_goal <= dist_to_goal_) {
  //     dist_to_goal_ = distance_to_goal;
  //   } else {
  //     is_goal_reached_ = true;
  //     return true;
  //   }
  // }
  if (distance_to_goal <= goal_tolerance_) {
	  is_goal_reached_ = true;
  } else {
	  is_goal_reached_ = false;
  }
  return true;
}

bool PurePursuitPlanner::transformPose(
	const std::string &frame, const geometry_msgs::PoseStamped &in_pose,
	geometry_msgs::PoseStamped &out_pose) const {
  if (in_pose.header.frame_id==frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform<geometry_msgs::PoseStamped>(
      in_pose, out_pose, frame, ros::Duration(transform_tolerance_));
    return true;
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("PurePursuitPlanner: Exception in transform pose - %s",
          ex.what());
  }
  return false;
}

bool PurePursuitPlanner::transformPlan(
	std::vector<geometry_msgs::PoseStamped> &transformed_plan) {
  geometry_msgs::PoseStamped robot_pose;
  if (!transformPose(plan_[0].header.frame_id, robot_pose_, robot_pose)) {
    ROS_ERROR("PurePursuitPlanner: Cannot transform robot pose");
    return false;
  }

  const costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
  const double max_transform_dist =
	  std::max(costmap->getSizeInMetersX(), costmap->getSizeInMetersY())/2.0;

  std::vector<geometry_msgs::PoseStamped>::iterator closest = plan_.begin();
  double min_dist_plan_robot = getEuclideanDistance(*closest, robot_pose);
  double dist_plan_robot;
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = plan_.begin();
	   it!=plan_.end(); it++) {
	  dist_plan_robot = getEuclideanDistance(*it, robot_pose);
	if (dist_plan_robot <= min_dist_plan_robot) {
	  closest = it;
	  min_dist_plan_robot = dist_plan_robot;
	}
  }

  std::vector<geometry_msgs::PoseStamped>::iterator furthest = plan_.end();
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = closest;
	   it!=plan_.end(); it++) {
    if (getEuclideanDistance(*it, robot_pose) > max_transform_dist) {
      furthest = it;
      break;
    }
  }

  geometry_msgs::PoseStamped stamped_point, transformed_point;
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = closest;
	   it!=furthest; it++) {
    stamped_point = *it;
    stamped_point.header.stamp = robot_pose.header.stamp;
    if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_point,
              transformed_point)) {
      ROS_ERROR("PurePursuitPlanner: Cannot transform plan point");
      return false;
    }
    transformed_plan.push_back(transformed_point);
  }

  plan_.erase(plan_.begin(), closest);

  if (transformed_plan.empty()) {
    ROS_ERROR("PurePursuitPlanner: Transformed plan is empty.");
    return false;
  }

  return true;
}

std::vector<geometry_msgs::PoseStamped> PurePursuitPlanner::getTargetPoints(
	const std::vector<geometry_msgs::PoseStamped> &plan, const double ahead_dist, const uint8_t size) const {
  std::vector<geometry_msgs::PoseStamped> poses;
  std::vector<geometry_msgs::PoseStamped>::const_iterator target_point =
	  plan.end();
  for (std::vector<geometry_msgs::PoseStamped>::const_iterator it = plan.begin();
	   it!=plan.end(); it++) {
    if (std::hypot((*it).pose.position.x, (*it).pose.position.y) >=
      ahead_dist) {
      target_point = it;
      break;
    }
  }
  if (target_point==plan.end()) {
	  target_point = std::prev(plan.end());
  }

  poses.push_back(*target_point);
  std::vector<geometry_msgs::PoseStamped>::const_iterator target = target_point;

  for (uint8_t i = 0; i < size; i++) {
    target = std::next(target);
    if (target==plan.end()) {
      break;
    }
    poses.push_back(*target);
  }

  return poses;
}

nav_msgs::Path PurePursuitPlanner::createPlanMsg(
	const std::vector<geometry_msgs::PoseStamped> &plan) const {
  nav_msgs::Path msg;
  msg.header.stamp = plan[0].header.stamp;
  msg.header.frame_id = plan[0].header.frame_id;
  msg.poses = plan;
  return msg;
}

geometry_msgs::PointStamped PurePursuitPlanner::createTargetMsg(
	const geometry_msgs::PoseStamped &target) const {
  geometry_msgs::PointStamped msg;
  msg.header.stamp = target.header.stamp;
  msg.header.frame_id = target.header.frame_id;
  msg.point.x = target.pose.position.x;
  msg.point.y = target.pose.position.y;
  msg.point.z = 0.1;
  return msg;
}

double PurePursuitPlanner::computeLinearVel(double angular_error,
											double actual_dist, double turning_radius, double goal_dist) {
  double vel = linear_vel_;
  if (std::fabs(angular_error) > decel_max_angular_error_) {
	  vel = safe_linear_vel_;
  }
  if (std::abs(goal_dist) <= lookahead_dist_) {
	  vel = safe_linear_vel_*std::abs(goal_dist)/std::abs(goal_dist);
  } else if (turning_radius < min_scaling_radius_) {
	  vel *= turning_radius/min_scaling_radius_;
  }
  return std::max(vel, min_scaling_vel_);
}

double PurePursuitPlanner::computeSteeringAngle(double angular_error,
												double actual_dist, double turning_radius, double goal_dist) const {
  if (std::abs(goal_dist) < lookahead_dist_) {
	  return 0.0;
  }
  if (angular_error > 1.57) {
	  return max_angular_pos_;
  } else if (angular_error < -1.57) {
	  return -max_angular_pos_;
  }
  return std::atan2(2*wheel_base_*std::sin(angular_error), actual_dist);
}

bool PurePursuitPlanner::imminentCollision(
	const geometry_msgs::Twist &cmd_vel, bool &closer_to_collision) {
  if (inCollision(robot_pose_)) {
    ROS_INFO("PurePursuitPlanner: Collision detected at current position");
    return true;
  }

  if (std::fabs(cmd_vel.linear.x) <= 0.01) {
	  return false;
  }

  std::vector<geometry_msgs::PoseStamped> collision_arc;
  geometry_msgs::PoseStamped collision_point;
  collision_point.header.stamp = robot_pose_.header.stamp;
  collision_point.header.frame_id = costmap_ros_->getGlobalFrameID();

  const double cell_time =
	  costmap_ros_->getCostmap()->getResolution()/cmd_vel.linear.x;

  double curr_x = robot_pose_.pose.position.x;
  double curr_y = robot_pose_.pose.position.y;
  double curr_theta =
	  tf2::getYaw<geometry_msgs::Quaternion>(robot_pose_.pose.orientation);
  size_t idx = 1;
  while (true) {
    double cost_time = idx*cell_time;
    bool out_of_decel_projected_time = cost_time > decel_projected_time_;
    bool out_of_collision_projected_time = cost_time > collision_projected_time_;
    if (out_of_decel_projected_time && out_of_collision_projected_time) {
      break;
    }
    curr_x += cell_time*cmd_vel.linear.x*cos(curr_theta);
    curr_y += cell_time*cmd_vel.linear.x*sin(curr_theta);
    curr_theta += cell_time*(cmd_vel.linear.x*std::tan(cmd_vel.angular.z))
      /wheel_base_;

    collision_point.pose.position.x = curr_x;
    collision_point.pose.position.y = curr_y;
    collision_point.pose.position.z = 0.1;
    collision_arc.push_back(collision_point);

    if (inCollision(collision_point)) {
      collision_arc_pub_.publish(createPlanMsg(collision_arc));
      if (!out_of_collision_projected_time) {
        ROS_INFO("PurePursuitPlanner: Collision detected at projected path");
        return true;
      }
      if (!out_of_decel_projected_time) {
        closer_to_collision = true;
      }
    }
    idx++;
  }
  collision_arc_pub_.publish(createPlanMsg(collision_arc));
  return false;
}

bool PurePursuitPlanner::inCollision(
	const geometry_msgs::PoseStamped &pose) const {
  unsigned int mx, my;
  const costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
  costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my);
  const unsigned char cost = costmap->getCost(mx, my);

  return (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
	  cost!=costmap_2d::NO_INFORMATION);
}
