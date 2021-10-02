#include <pcl/filters/median_filter.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//メディアンフィルターを行うときのwindowsize
//奇数のみ
#define WINDOW_SIZE 3
//メディアンフィルターによる点の移動距離の最大値
#define MAX_MOVE_DIS 100

class median_filter {
private:
  ros::Publisher point_pub0;
  ros::Subscriber point_sub0;
  ros::NodeHandle n;

  pcl::MedianFilter<pcl::PointXYZ> mf;
  //データがないことを表すnanを代入
  const float invalid = std::numeric_limits<float>::quiet_NaN();

public:
  median_filter() {
    // Publisher
    point_pub0 = n.advertise<sensor_msgs::PointCloud2>(
        "median_filter/PointCloud_MF0", 10);
    

    // Subscriber
    point_sub0 = n.subscribe("low_pass_filter/PointCloud_LPF0", 10,
                             &median_filter::MF_callback0, this);
    
  }

  void MF_callback0(const sensor_msgs::PointCloud2ConstPtr &point0) {
    //入力をpclで扱える形に変換
    pcl::PointCloud<pcl::PointXYZ> cloud0;
    pcl::fromROSMsg(*point0, cloud0);
    //点群のポイントを宣言
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0_t(
        new pcl::PointCloud<pcl::PointXYZ>(cloud0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0_out(
        new pcl::PointCloud<pcl::PointXYZ>(cloud0));
    //フィルターを行う点群を選択
    mf.setInputCloud(cloud0_t);
    //フィルターを行うサイズを指定
    mf.setWindowSize(WINDOW_SIZE);
    //点の移動距離の最大値を指定
    mf.setMaxAllowedMovement(MAX_MOVE_DIS);
    //フィルターを行う
    mf.filter(*cloud0_out);
    // publish
    point_pub0.publish(*cloud0_out);
    // ROS_INFO("median filtered0");
  }

};

int main(int argc, char **argv) {
  //初期化＆ノード名をmedian_filterに設定
  ros::init(argc, argv, "median_filtered2");

  //オブジェクト宣言
  median_filter median_filter;

  // Ctrl+Cが押されるまたはros::shutdown()が起こらない限りloop
  ros::spin();
  return 0;
}
