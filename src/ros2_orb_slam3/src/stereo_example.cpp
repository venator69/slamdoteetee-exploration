#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "orb_slam3/include/System.h"
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class StereoNode : public rclcpp::Node
{
public:
    StereoNode(ORB_SLAM3::System* pSLAM)
        : Node("orb_slam3_stereo_node"), mpSLAM(pSLAM)
    {
        RCLCPP_INFO(this->get_logger(), "Starting Stereo Node...");

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam3/pose", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/orbslam3/tracking_status", 10);
        compute_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("/orbslam3/compute_time_ms", 10);
        loop_status_pub_ = this->create_publisher<std_msgs::msg::String>("/orbslam3/loop_status", 10);

        auto qos_profile = rclcpp::SensorDataQoS().get_rmw_qos_profile();

        left_sub.subscribe(this, "/camera/camera/infra1/image_rect_raw", qos_profile);
        right_sub.subscribe(this, "/camera/camera/infra2/image_rect_raw", qos_profile);
        
        sync = std::make_shared<Synchronizer>(SyncPolicy(10), left_sub, right_sub);
        sync->registerCallback(std::bind(&StereoNode::GrabStereo, this, _1, _2));

        processing_thread = std::thread(&StereoNode::ProcessLoop, this);
    }

    ~StereoNode()
    {
        running = false;
        cv_data_ready.notify_all();
        if(processing_thread.joinable())
            processing_thread.join();
    }

private:
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    struct StereoFrame {
        cv::Mat left;
        cv::Mat right;
        double timestamp;
        rclcpp::Time ros_stamp;
    };

    void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr msgLeft,
                    const sensor_msgs::msg::Image::ConstSharedPtr msgRight)
    {
        try {
            cv::Mat imL = cv_bridge::toCvShare(msgLeft, "mono8")->image.clone();
            cv::Mat imR = cv_bridge::toCvShare(msgRight, "mono8")->image.clone();

            double tframe = msgLeft->header.stamp.sec + msgLeft->header.stamp.nanosec * 1e-9;

            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                if (frame_queue.size() >= 2) {
                    frame_queue.pop();
                }
                frame_queue.push({imL, imR, tframe, rclcpp::Time(msgLeft->header.stamp)});
            }
            cv_data_ready.notify_one();
        }
        catch(cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void ProcessLoop()
    {
        while(running && rclcpp::ok())
        {
            StereoFrame frame;
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                cv_data_ready.wait_for(lock, std::chrono::milliseconds(100), [this]{
                    return !frame_queue.empty() || !running;
                });

                if(!running || frame_queue.empty()) continue;

                frame = frame_queue.front();
                frame_queue.pop();
            }

            auto t1 = std::chrono::steady_clock::now();
            Sophus::SE3f Tcw = mpSLAM->TrackStereo(frame.left, frame.right, frame.timestamp);
            auto t2 = std::chrono::steady_clock::now();

            // Status Tracking
            int state = mpSLAM->GetTrackingState();
            std::string status_str;
            switch(state) {
                case 2:  status_str = "OK"; break;
                case 3:  status_str = "RECENTLY_LOST"; break;
                case 4:  status_str = "LOST"; break;
                default: status_str = "UNKNOWN";
            }

     
            bool bLoopDetected = false; 
            std::string loop_str = bLoopDetected ? "LOOP DETECTED!" : "No Loop";
            
            std_msgs::msg::String status_msg;
            status_msg.data = status_str;
            status_pub_->publish(status_msg);
            
            std_msgs::msg::String loop_msg;
            loop_msg.data = loop_str;
            loop_status_pub_->publish(loop_msg);

            double ttrack = std::chrono::duration<double, std::milli>(t2 - t1).count();
            std_msgs::msg::Float64 compute_msg;
            compute_msg.data = ttrack;
            compute_time_pub_->publish(compute_msg);
            RCLCPP_INFO(this->get_logger(), "Tracking: %s | Time: %.2f ms", status_str.c_str(), ttrack);

            if (!Tcw.matrix().isZero(0)) {
                PublishPose(Tcw, frame.ros_stamp);
            }
        }
    }

    void PublishPose(const Sophus::SE3f &Tcw, const rclcpp::Time &stamp)
    {
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f t = Twc.translation();
        Eigen::Quaternionf q(Twc.rotationMatrix());

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
        
        RCLCPP_INFO(this->get_logger(), 
        "POSE -> x: %.3f, y: %.3f, z: %.3f | qx: %.2f, qy: %.2f, qz: %.2f, qw:%.2f",
        t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
    }

    ORB_SLAM3::System* mpSLAM;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr compute_time_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr loop_status_pub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub;
    std::shared_ptr<Synchronizer> sync;
    std::queue<StereoFrame> frame_queue;
    std::mutex queue_mutex;
    std::condition_variable cv_data_ready;
    std::thread processing_thread;
    bool running = true;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if(argc < 3) {
        std::cerr << "\nUsage: ros2 run pkg stereo_node vocab config\n";
        return 1;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, false);
    auto node = std::make_shared<StereoNode>(&SLAM);
    rclcpp::spin(node);

    SLAM.Shutdown();
    rclcpp::shutdown();
    return 0;
}
