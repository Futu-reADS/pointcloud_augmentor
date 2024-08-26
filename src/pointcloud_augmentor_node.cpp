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

      no_push_back_ = declare_parameter<bool>("no_push_back", true);

      printf("direction_z:%lf length:%lf pitch:%lf offset:%lf\n", direction_z_, length_, pitch_, offset_);

      augment_number_ = 0;
      for (double offset=offset_; offset <= offset_ + length_; offset += pitch_) {
        augment_number_++;
      }
      offset_vector_.resize(augment_number_);
      for (size_t i=0; i < augment_number_; i++) {
        offset_vector_[i].x = direction_x_*(offset_ + pitch_*i);
        offset_vector_[i].y = direction_y_*(offset_ + pitch_*i);
        offset_vector_[i].z = direction_z_*(offset_ + pitch_*i);
      }
      
      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos_pcl2 = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

      pub_pcl2_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "pointcloud2_out", qos_pcl2);
      if (no_push_back_) {
        sub_pcl2_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "pointcloud2_in", qos_pcl2,
          std::bind(&PointCloudAugmentorNode::callbackForPointcloud2NoPushBack, this, std::placeholders::_1));
      } else {
        sub_pcl2_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "pointcloud2_in", qos_pcl2,
          std::bind(&PointCloudAugmentorNode::callbackForPointcloud2NoPushBack, this, std::placeholders::_1));
      }
    }

    void callbackForPointcloud2(const sensor_msgs::msg::PointCloud2::SharedPtr _ptr_msg_in)
    {
      auto time_start = std::chrono::system_clock::now();

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

      auto time_end = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_time = (time_end - time_start);
      if (elapsed_time.count() < callback_proc_time_min) {
        callback_proc_time_min = elapsed_time.count();
      }
      if (elapsed_time.count() > callback_proc_time_max) {
        callback_proc_time_max = elapsed_time.count();
      }
      callback_proc_time_avg = (callback_proc_time_avg * callback_count + elapsed_time.count())/(callback_count+1);
      callback_count++;

      // publish
      pub_pcl2_->publish(msg_out);
    }

    void callbackForPointcloud2NoPushBack(const sensor_msgs::msg::PointCloud2::SharedPtr _ptr_msg_in)
    {
      auto time_start = std::chrono::system_clock::now();

      pcl::PCLPointCloud2 pcl_msg_in;
      pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
      //pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_pcl_pointcloud_out(ptr_pcl_pointcloud);
    
      // convert ros2 PointCloud2 data into manageable format
      pcl_conversions::toPCL(*_ptr_msg_in, pcl_msg_in);
      pcl::fromPCLPointCloud2(pcl_msg_in, *ptr_pcl_pointcloud);

      // augment points along the specified direction
      size_t size_original = ptr_pcl_pointcloud->size();
      if (0 < pitch_) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_pcl_pointcloud_out(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointXYZI point;
        ptr_pcl_pointcloud_out->resize(size_original*augment_number_);
        uint64_t k=0;
        for (size_t i=0; i < size_original; i++) {
          for (size_t j=0; j < augment_number_; j++) {
            point = ptr_pcl_pointcloud->at(i);
            point.x += offset_vector_[j].x;
            point.y += offset_vector_[j].y;
            point.z += offset_vector_[j].z; ///
            ptr_pcl_pointcloud_out->at(k++) = point;
          }
        }
        // convert back to ros2 PointCloud2
        sensor_msgs::msg::PointCloud2 msg_out;
        pcl::PCLPointCloud2 pcl_msg_out;
        pcl::toPCLPointCloud2(*ptr_pcl_pointcloud_out, pcl_msg_out);
        pcl_conversions::fromPCL(pcl_msg_out, msg_out);
  
        msg_out.header = _ptr_msg_in->header;

        auto time_end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_time = (time_end - time_start);
        if (elapsed_time.count() < callback_proc_time_min || callback_proc_time_min == 0) {
          callback_proc_time_min = elapsed_time.count();
        }
        if (elapsed_time.count() > callback_proc_time_max) {
          callback_proc_time_max = elapsed_time.count();
        }
        callback_proc_time_avg = (callback_proc_time_avg * callback_count + elapsed_time.count())/(callback_count+1);
        callback_count++;

        // publish
        pub_pcl2_->publish(msg_out);

      } else {
        pub_pcl2_->publish(*_ptr_msg_in);
      }
    }

  public:
    double direction_x_, direction_y_, direction_z_;
    double length_;
    double offset_;
    double pitch_;
    bool no_push_back_;

    uint32_t augment_number_;
    std::vector<pcl::PointXYZ> offset_vector_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl2_;

    double callback_proc_time_min = 0; 
    double callback_proc_time_max = 0; 
    double callback_proc_time_avg = 0; 
    uint32_t callback_count = 0; 
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  //std::shared_ptr<IfbDriver> 
  auto augment_pointcloud = std::make_shared<PointCloudAugmentorNode>();
  rclcpp::spin(augment_pointcloud);
  rclcpp::shutdown();

  printf("processing time per one topic: min:%lf[s] avg:%lf[s] max:%lf[s]", 
    augment_pointcloud->callback_proc_time_min,
    augment_pointcloud->callback_proc_time_avg,
    augment_pointcloud->callback_proc_time_max); 

  return 0;
}
