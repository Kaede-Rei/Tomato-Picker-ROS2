#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "tomato_picker_interfaces/srv/set_scene_enabled.hpp"

#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace tomato_picker_perception {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //



// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief RGB-D 点云预处理节点
 */
class CloudPreprocessor final : public rclcpp::Node {
public:
    using Image = sensor_msgs::msg::Image;
    using CameraInfo = sensor_msgs::msg::CameraInfo;
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image, CameraInfo>;
    using SetSceneEnabled = tomato_picker_interfaces::srv::SetSceneEnabled;

    /**
     * @brief 构造点云预处理节点并初始化订阅、发布与场景控制服务
     */
    CloudPreprocessor()
        : Node("cloud_preprocessor"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_) {
        declare_parameters();
        load_parameters();

        raw_pub_ = create_publisher<PointCloud2>(raw_topic_, rclcpp::SensorDataQoS());
        base_pub_ = create_publisher<PointCloud2>(base_topic_, rclcpp::SensorDataQoS());
        filtered_pub_ = create_publisher<PointCloud2>(filtered_topic_, rclcpp::SensorDataQoS());

        color_sub_.subscribe(this, color_topic_, rmw_qos_profile_sensor_data);
        depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
        depth_info_sub_.subscribe(this, depth_info_topic_, rmw_qos_profile_sensor_data);

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), color_sub_, depth_sub_, depth_info_sub_);
        sync_->registerCallback(std::bind(&CloudPreprocessor::image_callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        clear_octomap_client_ = create_client<std_srvs::srv::Empty>(clear_octomap_service_);
        scene_service_ = create_service<SetSceneEnabled>(
            scene_service_name_,
            std::bind(&CloudPreprocessor::scene_service_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "CloudPreprocessor started; color=%s depth=%s info=%s", color_topic_.c_str(), depth_topic_.c_str(), depth_info_topic_.c_str());
    }

private:
    /**
     * @brief 声明节点参数
     */
    void declare_parameters() {
        declare_parameter<std::string>("topics.color_image", "/camera/wrist/color/image_raw");
        declare_parameter<std::string>("topics.depth_image", "/camera/wrist/depth_registered/image_raw");
        declare_parameter<std::string>("topics.depth_info", "/camera/wrist/depth_registered/camera_info");
        declare_parameter<std::string>("topics.cloud_raw", "/tomato_picker/perception/cloud/raw");
        declare_parameter<std::string>("topics.cloud_base", "/tomato_picker/perception/cloud/base");
        declare_parameter<std::string>("topics.cloud_filtered", "/tomato_picker/perception/cloud/filtered");
        declare_parameter<std::string>("topics.clear_octomap_service", "/clear_octomap");
        declare_parameter<std::string>("topics.scene_service", "/tomato_picker/perception/set_scene_enabled");
        declare_parameter<std::string>("target_frame", "base_link");
        declare_parameter<int>("pixel_stride", 4);
        declare_parameter<int>("frame_skip", 1);
        declare_parameter<double>("min_depth", 0.15);
        declare_parameter<double>("max_depth", 1.50);
        declare_parameter<double>("workspace.min_x", 0.0);
        declare_parameter<double>("workspace.max_x", 0.8);
        declare_parameter<double>("workspace.min_y", -0.5);
        declare_parameter<double>("workspace.max_y", 0.5);
        declare_parameter<double>("workspace.min_z", 0.0);
        declare_parameter<double>("workspace.max_z", 1.2);
        declare_parameter<double>("voxel_leaf", 0.015);
        declare_parameter<int>("sor_mean_k", 20);
        declare_parameter<double>("sor_stddev", 1.0);
        declare_parameter<bool>("publish_raw", true);
        declare_parameter<bool>("publish_base", true);
        declare_parameter<bool>("publish_filtered", true);
    }

    /**
     * @brief 从 ROS 2 参数服务器加载运行配置
     */
    void load_parameters() {
        color_topic_ = get_parameter("topics.color_image").as_string();
        depth_topic_ = get_parameter("topics.depth_image").as_string();
        depth_info_topic_ = get_parameter("topics.depth_info").as_string();
        raw_topic_ = get_parameter("topics.cloud_raw").as_string();
        base_topic_ = get_parameter("topics.cloud_base").as_string();
        filtered_topic_ = get_parameter("topics.cloud_filtered").as_string();
        clear_octomap_service_ = get_parameter("topics.clear_octomap_service").as_string();
        scene_service_name_ = get_parameter("topics.scene_service").as_string();
        target_frame_ = get_parameter("target_frame").as_string();
        pixel_stride_ = get_parameter("pixel_stride").as_int();
        frame_skip_ = get_parameter("frame_skip").as_int();
        min_depth_ = get_parameter("min_depth").as_double();
        max_depth_ = get_parameter("max_depth").as_double();
        min_x_ = get_parameter("workspace.min_x").as_double();
        max_x_ = get_parameter("workspace.max_x").as_double();
        min_y_ = get_parameter("workspace.min_y").as_double();
        max_y_ = get_parameter("workspace.max_y").as_double();
        min_z_ = get_parameter("workspace.min_z").as_double();
        max_z_ = get_parameter("workspace.max_z").as_double();
        voxel_leaf_ = get_parameter("voxel_leaf").as_double();
        sor_mean_k_ = get_parameter("sor_mean_k").as_int();
        sor_stddev_ = get_parameter("sor_stddev").as_double();
        publish_raw_ = get_parameter("publish_raw").as_bool();
        publish_base_ = get_parameter("publish_base").as_bool();
        publish_filtered_ = get_parameter("publish_filtered").as_bool();
    }

    /**
     * @brief 同步处理彩色图、注册深度图和相机内参
     * @param color_msg 彩色图像
     * @param depth_msg 注册到彩色图的深度图像
     * @param depth_info_msg 深度图相机内参
     */
    void image_callback(const Image::ConstSharedPtr& color_msg, const Image::ConstSharedPtr& depth_msg, const CameraInfo::ConstSharedPtr& depth_info_msg) {
        if(frame_skip_ > 1 && (frame_count_++ % frame_skip_ != 0)) return;

        cv_bridge::CvImageConstPtr color_cv_ptr;
        cv_bridge::CvImageConstPtr depth_cv_ptr;
        try {
            color_cv_ptr = cv_bridge::toCvShare(color_msg, "bgr8");
            depth_cv_ptr = cv_bridge::toCvShare(depth_msg);
        }
        catch(const cv_bridge::Exception& error) {
            RCLCPP_WARN(get_logger(), "cv_bridge error: %s", error.what());
            return;
        }

        if(depth_cv_ptr->image.type() != CV_32FC1) {
            RCLCPP_WARN(get_logger(), "unsupported depth image type: %d", depth_cv_ptr->image.type());
            return;
        }

        const auto& color = color_cv_ptr->image;
        const auto& depth = depth_cv_ptr->image;
        if(color.rows != depth.rows || color.cols != depth.cols) {
            RCLCPP_WARN(get_logger(), "color/depth size mismatch: color=%dx%d depth=%dx%d", color.cols, color.rows, depth.cols, depth.rows);
            return;
        }

        const double fx = depth_info_msg->k[0];
        const double fy = depth_info_msg->k[4];
        const double cx = depth_info_msg->k[2];
        const double cy = depth_info_msg->k[5];
        if(fx <= 0.0 || fy <= 0.0) {
            RCLCPP_WARN(get_logger(), "invalid camera intrinsics");
            return;
        }

        auto raw_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        raw_cloud->is_dense = false;
        raw_cloud->points.reserve(static_cast<std::size_t>((depth.rows / pixel_stride_ + 1) * (depth.cols / pixel_stride_ + 1)));

        for(int v = 0; v < depth.rows; v += pixel_stride_) {
            for(int u = 0; u < depth.cols; u += pixel_stride_) {
                const float z = depth.at<float>(v, u);
                if(!std::isfinite(z) || z < min_depth_ || z > max_depth_) continue;

                const auto& bgr = color.at<cv::Vec3b>(v, u);
                pcl::PointXYZRGB point;
                point.x = static_cast<float>((static_cast<double>(u) - cx) * z / fx);
                point.y = static_cast<float>((static_cast<double>(v) - cy) * z / fy);
                point.z = z;
                point.b = bgr[0];
                point.g = bgr[1];
                point.r = bgr[2];
                raw_cloud->points.push_back(point);
            }
        }
        raw_cloud->width = static_cast<std::uint32_t>(raw_cloud->points.size());
        raw_cloud->height = 1;

        PointCloud2 raw_msg;
        pcl::toROSMsg(*raw_cloud, raw_msg);
        raw_msg.header = depth_msg->header;
        if(publish_raw_) raw_pub_->publish(raw_msg);

        PointCloud2 base_msg = raw_msg;
        if(!target_frame_.empty() && raw_msg.header.frame_id != target_frame_) {
            try {
                const auto transform = tf_buffer_.lookupTransform(target_frame_, raw_msg.header.frame_id, rclcpp::Time(raw_msg.header.stamp), rclcpp::Duration::from_seconds(0.1));
                tf2::doTransform(raw_msg, base_msg, transform);
                base_msg.header.frame_id = target_frame_;
            }
            catch(const tf2::TransformException& error) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF transform failed: %s", error.what());
                return;
            }
        }
        if(publish_base_) base_pub_->publish(base_msg);

        auto base_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        auto filtered_x = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        auto filtered_y = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        auto filtered_z = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        auto voxel_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        auto final_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::fromROSMsg(base_msg, *base_cloud);

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(base_cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(min_x_, max_x_);
        pass.filter(*filtered_x);
        pass.setInputCloud(filtered_x);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(min_y_, max_y_);
        pass.filter(*filtered_y);
        pass.setInputCloud(filtered_y);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_z_, max_z_);
        pass.filter(*filtered_z);

        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        voxel.setInputCloud(filtered_z);
        voxel.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
        voxel.filter(*voxel_cloud);

        if(static_cast<int>(voxel_cloud->points.size()) > sor_mean_k_) {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
            sor.setInputCloud(voxel_cloud);
            sor.setMeanK(sor_mean_k_);
            sor.setStddevMulThresh(sor_stddev_);
            sor.filter(*final_cloud);
        }
        else {
            final_cloud = voxel_cloud;
        }

        final_cloud->width = static_cast<std::uint32_t>(final_cloud->points.size());
        final_cloud->height = 1;
        final_cloud->is_dense = false;

        PointCloud2 filtered_msg;
        pcl::toROSMsg(*final_cloud, filtered_msg);
        filtered_msg.header = base_msg.header;
        if(publish_filtered_ && scene_enabled_.load()) filtered_pub_->publish(filtered_msg);
    }

    /**
     * @brief 开关规划场景点云并可请求 MoveIt 2 清空 Octomap
     * @param request 场景控制请求
     * @param response 场景控制结果
     */
    void scene_service_callback(const SetSceneEnabled::Request::SharedPtr request, SetSceneEnabled::Response::SharedPtr response) {
        scene_enabled_.store(request->enabled);
        response->success = true;
        response->message = request->enabled ? "scene enabled" : "scene disabled";

        if(!request->clear_octomap) return;
        if(!clear_octomap_client_->service_is_ready()) {
            response->success = false;
            response->message += "; clear_octomap service is unavailable";
            return;
        }

        auto clear_request = std::make_shared<std_srvs::srv::Empty::Request>();
        clear_octomap_client_->async_send_request(clear_request);
        response->message += "; clear_octomap requested";
    }

private:
    message_filters::Subscriber<Image> color_sub_;             ///< 彩色图订阅器
    message_filters::Subscriber<Image> depth_sub_;             ///< 注册深度图订阅器
    message_filters::Subscriber<CameraInfo> depth_info_sub_;   ///< 相机内参订阅器
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;  ///< 三路近似时间同步器

    rclcpp::Publisher<PointCloud2>::SharedPtr raw_pub_;         ///< 相机坐标系原始点云发布器
    rclcpp::Publisher<PointCloud2>::SharedPtr base_pub_;        ///< 规划参考系点云发布器
    rclcpp::Publisher<PointCloud2>::SharedPtr filtered_pub_;    ///< 过滤后规划场景点云发布器
    rclcpp::Service<SetSceneEnabled>::SharedPtr scene_service_; ///< 场景开关服务
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_octomap_client_;  ///< MoveIt 2 Octomap 清图客户端

    tf2_ros::Buffer tf_buffer_;                    ///< TF 缓冲区
    tf2_ros::TransformListener tf_listener_;       ///< TF 监听器
    std::atomic_bool scene_enabled_{ true };       ///< 是否发布规划场景点云

    std::string color_topic_;                      ///< 彩色图 topic
    std::string depth_topic_;                      ///< 注册深度图 topic
    std::string depth_info_topic_;                 ///< 相机内参 topic
    std::string raw_topic_;                        ///< 原始点云 topic
    std::string base_topic_;                       ///< 参考系点云 topic
    std::string filtered_topic_;                   ///< 过滤点云 topic
    std::string clear_octomap_service_;            ///< MoveIt 2 清图服务名
    std::string scene_service_name_;               ///< 场景控制服务名
    std::string target_frame_;                     ///< 点云目标参考系

    int pixel_stride_{ 4 };                        ///< 像素降采样步长
    int frame_skip_{ 1 };                          ///< 图像帧跳过倍率
    int frame_count_{ 0 };                         ///< 已接收帧计数
    double min_depth_{ 0.15 };                     ///< 最小有效深度 m
    double max_depth_{ 1.50 };                     ///< 最大有效深度 m
    double min_x_{ 0.0 };                          ///< 工作空间 X 下界 m
    double max_x_{ 0.8 };                          ///< 工作空间 X 上界 m
    double min_y_{ -0.5 };                         ///< 工作空间 Y 下界 m
    double max_y_{ 0.5 };                          ///< 工作空间 Y 上界 m
    double min_z_{ 0.0 };                          ///< 工作空间 Z 下界 m
    double max_z_{ 1.2 };                          ///< 工作空间 Z 上界 m
    double voxel_leaf_{ 0.015 };                   ///< VoxelGrid 栅格尺寸 m
    int sor_mean_k_{ 20 };                         ///< SOR 邻域点数量
    double sor_stddev_{ 1.0 };                     ///< SOR 标准差阈值
    bool publish_raw_{ true };                     ///< 是否发布原始点云
    bool publish_base_{ true };                    ///< 是否发布参考系点云
    bool publish_filtered_{ true };                ///< 是否发布过滤点云
};

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //

} // namespace tomato_picker_perception

/**
 * @brief 启动 RGB-D 点云预处理 capability 节点
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return 正常退出返回 0
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tomato_picker_perception::CloudPreprocessor>());
    rclcpp::shutdown();
    return 0;
}
