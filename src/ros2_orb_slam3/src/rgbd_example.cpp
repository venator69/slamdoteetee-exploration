#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "orb_slam3/include/System.h"
#include <opencv2/opencv.hpp>
#include <deque>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class RgbdNode : public rclcpp::Node
{
public:
    RgbdNode(ORB_SLAM3::System* pSLAM, bool use_imu, bool use_drift_correction)
        : Node("orb_slam3_rgbd_node"), mpSLAM(pSLAM), use_imu_(use_imu), use_drift_correction_(use_drift_correction)
    {
        min_frame_interval_sec_ = declare_parameter("min_frame_interval_sec", 0.125);
        min_pose_travel_m_ = declare_parameter("min_pose_travel_m", 0.0);
        rgb_topic_ = declare_parameter<std::string>(
            "rgb_topic", "/camera/camera/color/image_raw");
        depth_topic_ = declare_parameter<std::string>(
            "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
        imu_topic_ = declare_parameter<std::string>(
            "imu_topic", "/camera/camera/imu");

        declare_parameter("use_imu", use_imu_);
        declare_parameter("use_drift_correction", use_drift_correction_);

        RCLCPP_INFO(
            this->get_logger(),
            "RGBD node: rgb=%s depth=%s imu=%s use_imu=%s drift=%s frame_interval=%.3fs",
            rgb_topic_.c_str(), depth_topic_.c_str(), imu_topic_.c_str(),
            use_imu_ ? "on" : "off", use_drift_correction_ ? "on" : "off",
            min_frame_interval_sec_);

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam3/pose", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>("/orbslam3/tracking_status", 10);
        compute_time_pub_ = create_publisher<std_msgs::msg::Float64>("/orbslam3/compute_time_ms", 10);
        loop_status_pub_ = create_publisher<std_msgs::msg::String>("/orbslam3/loop_status", 10);
        drift_risk_pub_ = create_publisher<std_msgs::msg::Float64>("/orbslam3/drift_risk_score", 10);
        drift_error_pub_ = create_publisher<std_msgs::msg::Float64>("/orbslam3/drift_predicted_error", 10);
        imu_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/orbslam3/imu_enabled", 10);
        drift_enabled_pub_ = create_publisher<std_msgs::msg::Bool>("/orbslam3/drift_correction_enabled", 10);

        auto qos_profile = rclcpp::SensorDataQoS().get_rmw_qos_profile();
        rgb_sub_.subscribe(this, rgb_topic_, qos_profile);
        depth_sub_.subscribe(this, depth_topic_, qos_profile);

        sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), rgb_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&RgbdNode::GrabRGBD, this, _1, _2));

        if (use_imu_) {
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic_, rclcpp::SensorDataQoS(),
                std::bind(&RgbdNode::GrabImu, this, _1));
        }

        imu_srv_ = create_service<std_srvs::srv::SetBool>(
            "/orbslam3/set_imu_enabled",
            std::bind(&RgbdNode::HandleSetImu, this, _1, _2));
        drift_srv_ = create_service<std_srvs::srv::SetBool>(
            "/orbslam3/set_drift_correction",
            std::bind(&RgbdNode::HandleSetDrift, this, _1, _2));

        param_cb_ = add_on_set_parameters_callback(
            std::bind(&RgbdNode::OnParameters, this, std::placeholders::_1));

        publishPipelineState();

        processing_thread_ = std::thread(&RgbdNode::ProcessLoop, this);
    }

    ~RgbdNode()
    {
        running_ = false;
        cv_data_ready_.notify_all();
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    struct RgbdFrame {
        cv::Mat rgb;
        cv::Mat depth;
        double timestamp;
        rclcpp::Time ros_stamp;
    };

    static bool IsValidPose(const Sophus::SE3f& Tcw)
    {
        return !Tcw.matrix().isZero(0) && std::isfinite(Tcw.translation().x());
    }

    void publishPipelineState()
    {
        std_msgs::msg::Bool imu_msg;
        imu_msg.data = use_imu_;
        imu_enabled_pub_->publish(imu_msg);

        std_msgs::msg::Bool drift_msg;
        drift_msg.data = use_drift_correction_;
        drift_enabled_pub_->publish(drift_msg);
    }

    void publishDriftMetrics()
    {
        std_msgs::msg::Float64 risk_msg;
        risk_msg.data = 0.0;
        drift_risk_pub_->publish(risk_msg);

        std_msgs::msg::Float64 error_msg;
        error_msg.data = 0.0;
        drift_error_pub_->publish(error_msg);
    }

    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!use_imu_) {
            return;
        }

        const double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        ORB_SLAM3::IMU::Point sample(
            msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
            msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
            t);

        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_buffer_.push_back(sample);
        while (imu_buffer_.size() > 4000) {
            imu_buffer_.pop_front();
        }
    }

    std::vector<ORB_SLAM3::IMU::Point> CollectImuMeasurements(double frame_time)
    {
        std::vector<ORB_SLAM3::IMU::Point> measurements;
        if (!use_imu_) {
            return measurements;
        }

        std::lock_guard<std::mutex> lock(imu_mutex_);
        const double start_time = has_last_frame_time_ ? last_frame_time_ : (frame_time - 0.5);

        while (!imu_buffer_.empty() && imu_buffer_.front().t < start_time) {
            imu_buffer_.pop_front();
        }

        for (const auto& sample : imu_buffer_) {
            if (sample.t > frame_time) {
                break;
            }
            if (sample.t >= start_time) {
                measurements.push_back(sample);
            }
        }

        return measurements;
    }

    void GrabRGBD(const sensor_msgs::msg::Image::ConstSharedPtr msgRgb,
                  const sensor_msgs::msg::Image::ConstSharedPtr msgDepth)
    {
        try {
            cv::Mat rgb = cv_bridge::toCvShare(msgRgb, "bgr8")->image.clone();
            cv::Mat depth = cv_bridge::toCvShare(msgDepth)->image.clone();
            double tframe = msgRgb->header.stamp.sec + msgRgb->header.stamp.nanosec * 1e-9;

            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                while (!frame_queue_.empty()) {
                    frame_queue_.pop();
                }
                frame_queue_.push({rgb, depth, tframe, rclcpp::Time(msgRgb->header.stamp)});
            }
            cv_data_ready_.notify_one();
        }
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void ProcessLoop()
    {
        while (running_ && rclcpp::ok()) {
            RgbdFrame frame;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_data_ready_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                    return !frame_queue_.empty() || !running_;
                });

                if (!running_ || frame_queue_.empty()) {
                    continue;
                }

                frame = frame_queue_.front();
                frame_queue_.pop();
            }

            const auto now = std::chrono::steady_clock::now();
            if (last_processed_.time_since_epoch().count() != 0) {
                const double elapsed =
                    std::chrono::duration<double>(now - last_processed_).count();
                if (elapsed < min_frame_interval_sec_) {
                    continue;
                }
            }
            last_processed_ = now;

            const std::vector<ORB_SLAM3::IMU::Point> imu_meas =
                CollectImuMeasurements(frame.timestamp);

            const auto t1 = std::chrono::steady_clock::now();
            Sophus::SE3f Tcw = mpSLAM->TrackRGBD(frame.rgb, frame.depth, frame.timestamp, imu_meas);
            const auto t2 = std::chrono::steady_clock::now();

            last_frame_time_ = frame.timestamp;
            has_last_frame_time_ = true;

            const int state = mpSLAM->GetTrackingState();
            std::string status_str;
            switch (state) {
                case 2: status_str = "OK"; break;
                case 3: status_str = "RECENTLY_LOST"; break;
                case 4: status_str = "LOST"; break;
                default: status_str = "UNKNOWN"; break;
            }

            std_msgs::msg::String status_msg;
            status_msg.data = status_str;
            status_pub_->publish(status_msg);

            std_msgs::msg::String loop_msg;
            loop_msg.data = "No Loop";
            loop_status_pub_->publish(loop_msg);

            const double ttrack = std::chrono::duration<double, std::milli>(t2 - t1).count();
            std_msgs::msg::Float64 compute_msg;
            compute_msg.data = ttrack;
            compute_time_pub_->publish(compute_msg);
            publishDriftMetrics();

            Sophus::SE3f pose_Tcw;
            bool publish_pose = false;

            if (IsValidPose(Tcw)) {
                pose_Tcw = Tcw;
                last_good_Tcw_ = Tcw;
                has_last_good_pose_ = true;
                publish_pose = true;
            } else if (status_str == "RECENTLY_LOST") {
                Sophus::SE3f Twc = mpSLAM->GetCamTwc();
                if (IsValidPose(Twc.inverse())) {
                    pose_Tcw = Twc.inverse();
                    publish_pose = true;
                } else if (has_last_good_pose_) {
                    pose_Tcw = last_good_Tcw_;
                    publish_pose = true;
                }
            }

            if (publish_pose) {
                if (ShouldPublishPose(pose_Tcw)) {
                    PublishPose(pose_Tcw, frame.ros_stamp);
                    last_published_Tcw_ = pose_Tcw;
                    has_last_published_pose_ = true;
                }
            }

            if (status_str != last_logged_status_ ||
                std::chrono::duration<double>(now - last_status_log_).count() >= 2.0) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Tracking: %s | Time: %.2f ms | Pose: %s | IMU samples: %zu | Drift risk: %.3f",
                    status_str.c_str(), ttrack, publish_pose ? "yes" : "no",
                    imu_meas.size(), 0.0);
                last_logged_status_ = status_str;
                last_status_log_ = now;
            }
        }
    }

    bool ShouldPublishPose(const Sophus::SE3f& Tcw) const
    {
        if (min_pose_travel_m_ <= 0.0 || !has_last_published_pose_) {
            return true;
        }

        const Eigen::Vector3f delta =
            Tcw.inverse().translation() - last_published_Tcw_.inverse().translation();
        return delta.norm() >= min_pose_travel_m_;
    }

    void PublishPose(const Sophus::SE3f& Tcw, const rclcpp::Time& stamp)
    {
        const Sophus::SE3f Twc = Tcw.inverse();
        const Eigen::Vector3f t = Twc.translation();
        const Eigen::Quaternionf q(Twc.rotationMatrix());

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = t.x();
        pose_msg.pose.position.y = t.y();
        pose_msg.pose.position.z = t.z();
        pose_msg.pose.orientation.x = q.x();
        pose_msg.pose.orientation.y = q.y();
        pose_msg.pose.orientation.z = q.z();
        pose_msg.pose.orientation.w = q.w();
        pose_pub_->publish(pose_msg);

        RCLCPP_INFO(
            this->get_logger(),
            "POSE -> x: %.3f, y: %.3f, z: %.3f | qx: %.2f, qy: %.2f, qz: %.2f, qw: %.2f",
            t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
    }

    void HandleSetImu(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        use_imu_ = request->data;
        if (!use_imu_) {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            imu_buffer_.clear();
        }
        publishPipelineState();
        response->success = true;
        response->message = use_imu_ ? "IMU fusion enabled" : "IMU fusion disabled";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    void HandleSetDrift(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        use_drift_correction_ = request->data;
        publishPipelineState();
        response->success = true;
        response->message = use_drift_correction_
            ? "Drift correction enabled"
            : "Drift correction disabled";
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    rcl_interfaces::msg::SetParametersResult OnParameters(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
            if (param.get_name() == "use_imu") {
                use_imu_ = param.as_bool();
                if (!use_imu_) {
                    std::lock_guard<std::mutex> lock(imu_mutex_);
                    imu_buffer_.clear();
                }
            } else if (param.get_name() == "use_drift_correction") {
                use_drift_correction_ = param.as_bool();
            }
        }

        publishPipelineState();
        return result;
    }

    ORB_SLAM3::System* mpSLAM;
    std::string rgb_topic_;
    std::string depth_topic_;
    std::string imu_topic_;
    bool use_imu_;
    bool use_drift_correction_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr compute_time_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr loop_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drift_risk_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr drift_error_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr imu_enabled_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drift_enabled_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr imu_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr drift_srv_;
    OnSetParametersCallbackHandle::SharedPtr param_cb_;

    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<Synchronizer> sync_;
    std::queue<RgbdFrame> frame_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_data_ready_;
    std::thread processing_thread_;
    bool running_ = true;

    std::deque<ORB_SLAM3::IMU::Point> imu_buffer_;
    std::mutex imu_mutex_;
    double last_frame_time_ = 0.0;
    bool has_last_frame_time_ = false;

    double min_frame_interval_sec_ = 0.125;
    double min_pose_travel_m_ = 0.0;

    std::chrono::steady_clock::time_point last_processed_{};
    std::chrono::steady_clock::time_point last_status_log_{};
    std::string last_logged_status_;

    bool has_last_good_pose_ = false;
    Sophus::SE3f last_good_Tcw_;
    bool has_last_published_pose_ = false;
    Sophus::SE3f last_published_Tcw_;
};

static bool ConfigHasImuSection(const std::string& config_path)
{
    cv::FileStorage fs(config_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return false;
    }
    const bool has_freq = !fs["IMU.Frequency"].empty();
    const bool has_noise = !fs["IMU.NoiseGyro"].empty();
    return has_freq && has_noise;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (argc < 3) {
        std::cerr << "\nUsage: ros2 run pkg rgbd_node_cpp vocab config\n";
        return 1;
    }

    auto param_node = std::make_shared<rclcpp::Node>("rgbd_param_loader");
    const bool config_has_imu = ConfigHasImuSection(argv[2]);
    bool use_imu = param_node->declare_parameter("use_imu", config_has_imu);
    bool use_drift_correction = param_node->declare_parameter("use_drift_correction", false);

    if (use_imu && !config_has_imu) {
        RCLCPP_WARN(
            param_node->get_logger(),
            "use_imu requested but config has no IMU section — falling back to RGB-D mode");
        use_imu = false;
    }

    const auto sensor = use_imu
        ? ORB_SLAM3::System::IMU_RGBD
        : ORB_SLAM3::System::RGBD;

    ORB_SLAM3::System SLAM(argv[1], argv[2], sensor, false);

    auto node = std::make_shared<RgbdNode>(&SLAM, use_imu, use_drift_correction);
    rclcpp::spin(node);

    SLAM.Shutdown();
    rclcpp::shutdown();
    return 0;
}
