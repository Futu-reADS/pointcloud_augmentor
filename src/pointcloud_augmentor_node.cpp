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
    Node("pointcloud_augmentor_node"),
    clear_points_(false)
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
      horizontal_multiplier_ = 32;
            
      no_push_back_ = declare_parameter<bool>("no_push_back", true);
      clear_points_ = declare_parameter("clear_points", clear_points_);

      printf("direction_z:%lf length:%lf pitch:%lf offset:%lf\n", direction_z_, length_, pitch_, offset_);

      update_offset_vector();
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

      param_cb_ = add_on_set_parameters_callback(
        std::bind(&PointCloudAugmentorNode::callbackOnParameterChange, this, std::placeholders::_1));

    }

    // obsolete; use callbackForPointCloud2NoPushBack instead
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
        if (!clear_points_) {
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
        if (clear_points_) {
          ptr_pcl_pointcloud_out->resize(0);
        } else {
          ptr_pcl_pointcloud_out->resize(size_original*augment_number_*horizontal_multiplier_);
          uint64_t k=0;
          for (size_t i=0; i < size_original; i++) {
            point = ptr_pcl_pointcloud->at(i);
            //double norm = sqrt(point.y*point.y + point.x*point.x);
            //double dx = -point.y/norm/256/horizontal_multiplier_;
            //double dy =  point.x/norm/256/horizontal_multiplier_;
            int kk_center = horizontal_multiplier_/2;
            for (size_t kk=0; kk < horizontal_multiplier_; kk++) {
              for (size_t j=0; j < augment_number_; j++) {
                point = ptr_pcl_pointcloud->at(i);
                point.x += offset_vector_[j].x + 0.005*(kk);;
                point.y += offset_vector_[j].y + 0.005*(kk-kk_center);
                point.z += offset_vector_[j].z; ///
                ptr_pcl_pointcloud_out->at(k++) = point;
              }
            }
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

    // For dynamic reconfiguration of pointcloud_augmentor; you can modify augmentation parameters on the fly
    rcl_interfaces::msg::SetParametersResult callbackOnParameterChange(
      const std::vector<rclcpp::Parameter>& _params)
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;

      for (const rclcpp::Parameter & param : _params)
      {
        if (param.get_name() == "clear_points")
        {
          clear_points_ = param.get_value<bool>();
        }
        else if (param.get_name() == "length") {
          length_ = param.get_value<double>();
        }
        else if (param.get_name() == "offset") {
          offset_ = param.get_value<double>();
        }
        else if (param.get_name() == "pitch") {
          pitch_ = param.get_value<double>();
        }
        else if (param.get_name() == "direction.x") {
          direction_x_ = param.get_value<double>();
        }
        else if (param.get_name() == "direction.y") {
          direction_y_ = param.get_value<double>();
        }
        else if (param.get_name() == "direction.z") {
          direction_z_ = param.get_value<double>();
        }
      }
      update_offset_vector();
      return result;
    }

    void update_offset_vector()
    {
      if (pitch_ <= 0) {
        return;
      }
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
    }


  public:
    double direction_x_, direction_y_, direction_z_;
    double length_;
    double offset_;
    double pitch_;
    bool no_push_back_;
    bool clear_points_;
    uint32_t horizontal_multiplier_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

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
