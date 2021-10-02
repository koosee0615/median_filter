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
  ros::Publisher point_pub0, point_pub_zed, point_pub1, point_pub2;
  ros::Subscriber point_sub0, point_sub_zed, point_sub1, point_sub2;
  ros::NodeHandle n;

  pcl::MedianFilter<pcl::PointXYZ> mf;
  //データがないことを表すnanを代入
  const float invalid = std::numeric_limits<float>::quiet_NaN();

public:
  median_filter() {
    // Publisher
    point_pub0 = n.advertise<sensor_msgs::PointCloud2>(
        "median_filter/PointCloud_MF0", 10);
    point_pub_zed = n.advertise<sensor_msgs::PointCloud2>(
        "median_filter/PointCloud_MF_zed", 1);
    point_pub1 = n.advertise<sensor_msgs::PointCloud2>(
        "median_filter/PointCloud_MF1", 10);
    point_pub2 = n.advertise<sensor_msgs::PointCloud2>(
        "median_filter/PointCloud_MF2", 10);

    // Subscriber
    point_sub0 = n.subscribe("low_pass_filter/PointCloud_LPF0", 10,
                             &median_filter::MF_callback0, this);
    point_sub_zed = n.subscribe("low_pass_filter/PointCloud_LPF_zed", 1,
                                &median_filter::MF_zed_callback, this);
    point_sub1 = n.subscribe("low_pass_filter/PointCloud_LPF1", 10,
                             &median_filter::MF_callback1, this);
    point_sub2 = n.subscribe("low_pass_filter/PointCloud_LPF2", 10,
                             &median_filter::MF_callback2, this);
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

  void MF_zed_callback(const sensor_msgs::PointCloud2ConstPtr &point_zed) {
    //入力をpclで扱える形に変換
    pcl::PointCloud<pcl::PointXYZ> cloud_zed, cloud_zed_out;
    pcl::fromROSMsg(*point_zed, cloud_zed);
    //点群のポイントを宣言
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_zed_t(
        new pcl::PointCloud<pcl::PointXYZ>(cloud_zed));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_zed_out_t(
        new pcl::PointCloud<pcl::PointXYZ>(cloud_zed_out));
    //フィルターを行う点群を選択
    mf.setInputCloud(cloud_zed_t);
    //フィルターを行うサイズを指定
    mf.setWindowSize(WINDOW_SIZE);
    //点の移動距離の最大値を指定
    mf.setMaxAllowedMovement(MAX_MOVE_DIS);
    //フィルターを行う
    mf.filter(*cloud_zed_out_t);
    // publish
    point_pub_zed.publish(*cloud_zed_out_t);
    // ROS_INFO("median filtered0");
  }

  /**********************************************************************************************************************************************
  /関数名：MF_callback1
  /low_pass_filter/PointCloud_LPF1がsubscribeされたときのコールバック関数
  /入力のpoincloudに対してノイズを除去するmedian_filterを行う
  /処理後の点群をmedian_filter/PointCloud_MF1としてPublishする
  /入力：const sensor_msgs::PointCloud2ConstPtr& point1
  /出力：void
  ***********************************************************************************************************************************************/
  void MF_callback1(const sensor_msgs::PointCloud2ConstPtr &point1) {
    //入力をpclで扱える形に変換
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    pcl::fromROSMsg(*point1, cloud1);
    //点群のポイントを宣言
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_t(
        new pcl::PointCloud<pcl::PointXYZ>(cloud1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_out(
        new pcl::PointCloud<pcl::PointXYZ>(cloud1));
    //フィルターを行う点群を選択
    mf.setInputCloud(cloud1_t);
    //フィルターを行うサイズを指定
    mf.setWindowSize(WINDOW_SIZE);
    //点の移動距離の最大値を指定
    mf.setMaxAllowedMovement(MAX_MOVE_DIS);
    //フィルターを行う
    mf.filter(*cloud1_out);
    // ROS_INFO("median filtered1");
    // publish
    point_pub1.publish(*cloud1_out);
  }

  /**********************************************************************************************************************************************
  /関数名：MF_callback2
  /low_pass_filter/PointCloud_LPF2がsubscribeされたときのコールバック関数
  /入力のpoincloudに対してノイズを除去するmedian_filterを行う
  /処理後の点群をmedian_filter/PointCloud_MF2としてPublishする
  /入力：const sensor_msgs::PointCloud2ConstPtr& point1
  /出力：void
  ***********************************************************************************************************************************************/

  void MF_callback2(const sensor_msgs::PointCloud2ConstPtr &point1) {
    //入力をpclで扱える形に変換
    pcl::PointCloud<pcl::PointXYZ> cloud2;
    pcl::fromROSMsg(*point1, cloud2);
    //点群のポイントを宣言
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_t(
        new pcl::PointCloud<pcl::PointXYZ>(cloud2));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_out(
        new pcl::PointCloud<pcl::PointXYZ>(cloud2));
    //フィルターを行う点群を選択
    mf.setInputCloud(cloud2_t);
    //フィルターを行うサイズを指定
    mf.setWindowSize(WINDOW_SIZE);
    //点の移動距離の最大値を指定
    mf.setMaxAllowedMovement(MAX_MOVE_DIS);
    //フィルターを行う
    mf.filter(*cloud2_out);
    // publish
    point_pub2.publish(*cloud2_out);
    // ROS_INFO("median filtered");
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
