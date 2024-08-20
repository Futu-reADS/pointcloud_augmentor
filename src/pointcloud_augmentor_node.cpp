#include <cstdio>

#include <rclcpp/rclcpp.hpp>

#include <rmw/qos_profiles.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>


class PointCloudAugmentorNode : public rclcpp::Node
{
  public:
    PointCloudAugmentorNode() :
    Node("pointcloud_augmentor_node") 
    {
      direction_x_ = declare_parameter<double>("direction.x", 0.0);
      direction_y_ = declare_parameter<double>("direction.y", 0.0);
      direction_z_ = declare_parameter<double>("direction.z", 0.0);
      double sqnorm = direction_x_*direction_x_ + direction_y_*direction_y_ + direction_z_*direction_z_;
      if (sqnorm < 1e-6) {
        pitch_ = 0;
      } else {
        double norm = sqrt(sqnorm);
        direction_x_ /= norm;
        direction_y_ /= norm;
        direction_z_ /= norm;
      }

      length_ = declare_parameter<double>("length", 0.0);
      offset_ = declare_parameter<double>("offset", 0.0);
      pitch_  = declare_parameter<double>("pitch", 0.0);

      printf("direction_z:%lf length:%lf pitch:%lf offset:%lf\n", direction_z_, length_, pitch_, offset_);

      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos_pcl2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

      pub_pcl2_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "pointcloud2_out", qos_pcl2);
      sub_pcl2_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud2_in", qos_pcl2,
        std::bind(&PointCloudAugmentorNode::callbackForPointcloud2, this, std::placeholders::_1));
    }

    void callbackForPointcloud2(const sensor_msgs::msg::PointCloud2::SharedPtr _ptr_msg_in)
    {
      pcl::PCLPointCloud2 pcl_msg_in;
      pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
      //pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_pcl_pointcloud_out(ptr_pcl_pointcloud);
    
      // convert ros2 PointCloud2 data into manageable format
      pcl_conversions::toPCL(*_ptr_msg_in, pcl_msg_in);
      pcl::fromPCLPointCloud2(pcl_msg_in, *ptr_pcl_pointcloud);

      // augment points along the specified direction
      size_t size_original = ptr_pcl_pointcloud->size();
      if (0 < pitch_) {
        for (size_t i=0; i < size_original; i++) {
          for (double offset=offset_; offset <= offset_ + length_; offset += pitch_) {
            pcl::PointXYZI point(ptr_pcl_pointcloud->at(i));
            point.x += direction_x_ * offset;
            point.y += direction_y_ * offset;
            point.z += direction_z_ * offset;
            ptr_pcl_pointcloud->push_back(point);
          }
        }
      }

      // convert back to ros2 PointCloud2
      sensor_msgs::msg::PointCloud2 msg_out;
      pcl::PCLPointCloud2 pcl_msg_out;
      pcl::toPCLPointCloud2(*ptr_pcl_pointcloud, pcl_msg_out);
      pcl_conversions::fromPCL(pcl_msg_out, msg_out);
 
      msg_out.header = _ptr_msg_in->header;

      // publish
      pub_pcl2_->publish(msg_out);
    }

  public:
    double direction_x_, direction_y_, direction_z_;
    double length_;
    double offset_;
    double pitch_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl2_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  //std::shared_ptr<IfbDriver> 
  auto augment_pointcloud = std::make_shared<PointCloudAugmentorNode>();
  rclcpp::spin(augment_pointcloud);
  rclcpp::shutdown();
  return 0;
}

