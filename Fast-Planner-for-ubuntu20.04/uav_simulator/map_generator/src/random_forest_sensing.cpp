#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Eigen>
#include <random>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;

ros::Publisher _local_map_pub;
ros::Publisher _all_map_pub;
ros::Publisher click_map_pub_;
ros::Subscriber _odom_sub;

vector<double> _state;

int _obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;

bool _map_ok = false;
bool _has_odom = false;

int circle_num_;
double radius_l_, radius_h_, z_l_, z_h_;
double theta_;
uniform_real_distribution<double> rand_radius_;
uniform_real_distribution<double> rand_radius2_;
uniform_real_distribution<double> rand_theta_;
uniform_real_distribution<double> rand_z_;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

sensor_msgs::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

  const int arraySize = 105;
  const double num[arraySize] = {0.655,
0.923,
0,
1.148,
0.782,
0.378,
1.160,
0.688,
0.920,
0.981,
0.652,
0.735,
0.337,
1.152,
0.325,
1.117,
0.618,
0.537,
0.503,
1.126,
0.485,
0.610,
0.585,
1.154,
0.429,
0.907,
0.354,
0.781,
0.500,
0,
0.799,
0.555,
1.069,
0.339,
0.982,
0.400,
0.557,
0.464,
1.037,
0.976,
0,
1.130,
0.306,
0.417,
0.760,
0.774,
1.198,
0.931,
1.167,
0.815,
0.543,
0.955,
0.997,
0,
0.610,
0.426,
0.712,
0.660,
0.410,
0.644,
0.768,
0.849,
1.176,
0.435,
0.522,
0.453,
0.714,
0.673,
0.598,
0.634,
0.784,
1.155,
0,
0.973,
0.833,
1.131,
0.742,
0.347,
0.328,
0.722,
1.181,
0.936,
0.381,
1.001,
0.706,
0,
0.457,
0.523,
0.753,
0.568,
0.860,
0.814,
0.912,
0.507,
0.389,
0.945,
0.801,
1.119,
0.410,
0.459,
0.376,
1.094,
0.494,
0.647,
1.109
};

void FixedArrayMapGenerate() {
  pcl::PointXYZ pt_fixed;

  // 极坐标障碍物网格的参数
  int grid_size_x = 13;
  int grid_size_y = 7;
  double grid_spacing_x = _x_size / (grid_size_x + 1);
  double grid_spacing_y = _y_size / (grid_size_y + 1);

  // 在网格中生成极坐标障碍物
  for (int i = 0; i < grid_size_x; i++) {
    for (int j = 0; j < grid_size_y; j++) {
      // 计算障碍物的位置
      double x = _x_l + (i + 1) * grid_spacing_x;
      double y = _y_l + (j + 1) * grid_spacing_y;
      // 使用平均尺寸以保持一致性
      double w = num[i*7+j];  // 平均宽度
      std::cout << "w" << w << std::endl;
      // double h = (_h_l + _h_h) / 2;  // 平均高度 
      double h = 7.0;
      // 跳过距离初始位置或(19,0)太近的位置
      if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0 ||
          sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
        continue;
      }

      // 调整位置以与分辨率网格对齐
      // x = floor(x / _resolution) * _resolution + _resolution / 2.0;
      // y = floor(y / _resolution) * _resolution + _resolution / 2.0;

      int widNum = ceil(w / _resolution);

      // 为极坐标障碍物生成点
      for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
        for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
          int heiNum = ceil(h / _resolution);
          for (int t = -10; t < heiNum; t++) {
            pt_fixed.x = x + (r + 0.5) * _resolution + 1e-2;
            pt_fixed.y = y + (s + 0.5) * _resolution + 1e-2;
            pt_fixed.z = (t + 0.5) * _resolution + 1e-2;
            cloudMap.points.push_back(pt_fixed);
          }
        }
    }
  }

  // 圆形障碍物网格的参数
  int circle_num_ = 36;
  int circle_grid_size = sqrt(circle_num_);
  double circle_spacing_x = _x_size / (circle_grid_size + 1);
  double circle_spacing_y = _y_size / (circle_grid_size + 1);

  // 在网格中生成圆形障碍物
  for (int i = 0; i < circle_grid_size; i++) {
    for (int j = 0; j < circle_grid_size; j++) {
      // 计算圆形障碍物的位置
      double x = _x_l + (i + 1) * circle_spacing_x - 1;
      double y = _y_l + (j + 1) * circle_spacing_y + 0.5;
      double z = (z_l_ + z_h_) / 2;  // 平均z坐标

      // 跳过距离初始位置或(19,0)太近的位置
      if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 2.0 ||
          sqrt(pow(x - 19.0, 2) + pow(y - 0.0, 2)) < 2.0) {
        continue;
      }

      // 调整位置以与分辨率网格对齐
      x = floor(x / _resolution) * _resolution + _resolution / 2.0;
      y = floor(y / _resolution) * _resolution + _resolution / 2.0;
      z = floor(z / _resolution) * _resolution + _resolution / 2.0;

      Eigen::Vector3d translate(x, y, z);

      // 所有圆形障碍物的固定方向
      double theta = 0;
      Eigen::Matrix3d rotate;
      rotate << cos(theta), -sin(theta), 0.0, sin(theta), cos(theta), 0.0, 0, 0, 1;

      // 使用平均半径以保持一致性
      double radius1 = (radius_l_ + radius_h_) / 2;
      double radius2 = (radius_l_ + 1.2) / 2;

      // 为圆形障碍物生成点
      Eigen::Vector3d cpt;
      for (double angle = 0.0; angle < 6.282; angle += _resolution / 2) {
        cpt(0) = 0.0;
        cpt(1) = radius1 * cos(angle);
        cpt(2) = radius2 * sin(angle);

        // 膨胀圆形障碍物
        Eigen::Vector3d cpt_if;
        for (int ifx = -0; ifx <= 0; ++ifx)
          for (int ify = -0; ify <= 0; ++ify)
            for (int ifz = -0; ifz <= 0; ++ifz) {
              cpt_if = cpt + Eigen::Vector3d(ifx * _resolution, ify * _resolution,
                                             ifz * _resolution);
              cpt_if = rotate * cpt_if + Eigen::Vector3d(x, y, z);
              pt_fixed.x = cpt_if(0);
              pt_fixed.y = cpt_if(1);
              pt_fixed.z = cpt_if(2);
              cloudMap.push_back(pt_fixed);
            }
      }
    }
  }

  // 设置点云的属性
  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("完成生成固定数组地图");

  // 构建KD树以便快速搜索
  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}



void rcvOdometryCallbck(const nav_msgs::Odometry odom) {
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return;
  _has_odom = true;

  _state = {odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}

int i = 0;
void pubSensedPoints() {
  // if (i < 10) {
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);
  // }

  return;

  /* ---------- only publish points around current position ---------- */
  if (!_map_ok || !_has_odom) return;

  pcl::PointCloud<pcl::PointXYZ> localMap;

  pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;

  if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
    return;

  if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                  pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      pt = cloudMap.points[pointIdxRadiusSearch[i]];
      localMap.points.push_back(pt);
    }
  } else {
    ROS_ERROR("[Map server] No obstacles .");
    return;
  }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  _local_map_pub.publish(localMap_pcd);
}

void clickCallback(const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  double w = rand_w(eng);
  double h;
  pcl::PointXYZ pt_random;

  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  int widNum = ceil(w / _resolution);

  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      h = rand_h(eng);
      int heiNum = ceil(h / _resolution);
      for (int t = -1; t < heiNum; t++) {
        pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
        pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
        pt_random.z = (t + 0.5) * _resolution + 1e-2;
        clicked_cloud_.points.push_back(pt_random);
        cloudMap.points.push_back(pt_random);
      }
    }
  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;

  pcl::toROSMsg(clicked_cloud_, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  click_map_pub_.publish(localMap_pcd);

  cloudMap.width = cloudMap.points.size();

  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  _local_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
  _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);

  click_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);
  // ros::Subscriber click_sub = n.subscribe("/goal", 10, clickCallback);

  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/resolution", _resolution, 0.1);
  // n.param("map/circle_num", circle_num_, 30);

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 5.0);
  n.param("ObstacleShape/upper_hei", _h_h, 9.0);

  n.param("ObstacleShape/radius_l", radius_l_, 7.0);
  n.param("ObstacleShape/radius_h", radius_h_, 7.0);
  n.param("ObstacleShape/z_l", z_l_, 6.0);
  n.param("ObstacleShape/z_h", z_h_, 7.0);
  n.param("ObstacleShape/theta", theta_, 7.0);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/radius", _sense_rate, 10.0);

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  ros::Duration(0.5).sleep();

  FixedArrayMapGenerate();

  ros::Rate loop_rate(_sense_rate);

  while (ros::ok()) {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
  }
}


