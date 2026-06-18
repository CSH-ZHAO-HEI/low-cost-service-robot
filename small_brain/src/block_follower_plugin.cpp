/**
 * block_follower_plugin.cpp
 * Gazebo World Plugin — 在物理引擎內部直接跟隨指定 link，零 ROS round-trip 延遲。
 *
 * 訂閱：
 *   /block_follower/attach  (std_msgs/String)  格式: "robot::link,block,x,y,z"
 *   /block_follower/detach  (std_msgs/String)  任意字串即可
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <thread>
#include <sstream>
#include <vector>
#include <string>
#include <atomic>

namespace gazebo {

class BlockFollowerPlugin : public WorldPlugin {
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr /*sdf*/) override {
    world_ = world;

    if (!ros::isInitialized()) {
      int argc = 0;
      ros::init(argc, nullptr, "block_follower_plugin",
                ros::init_options::NoSigintHandler);
    }
    nh_.reset(new ros::NodeHandle());
    nh_->setCallbackQueue(&cb_queue_);

    attach_sub_ = nh_->subscribe("/block_follower/attach", 1,
                                 &BlockFollowerPlugin::OnAttach, this);
    detach_sub_ = nh_->subscribe("/block_follower/detach", 1,
                                 &BlockFollowerPlugin::OnDetach, this);

    ros_thread_ = std::thread([this]() {
      while (ros::ok()) {
        cb_queue_.callAvailable(ros::WallDuration(0.01));
      }
    });

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&BlockFollowerPlugin::OnUpdate, this));

    ROS_INFO("[BlockFollowerPlugin] 載入完成");
  }

  ~BlockFollowerPlugin() {
    attached_    = false;
    wake_countdown_ = 0;
    nh_->shutdown();
    if (ros_thread_.joinable()) ros_thread_.join();
  }

private:
  void OnAttach(const std_msgs::String::ConstPtr &msg) {
    std::vector<std::string> parts;
    std::stringstream ss(msg->data);
    std::string token;
    while (std::getline(ss, token, ',')) parts.push_back(token);

    if (parts.size() < 5) {
      ROS_WARN("[BlockFollowerPlugin] attach 格式錯誤: %s", msg->data.c_str());
      return;
    }
    std::string link_full  = parts[0];
    std::string block_name = parts[1];
    double bx = std::stod(parts[2]);
    double by = std::stod(parts[3]);
    double bz = std::stod(parts[4]);

    auto colon = link_full.find("::");
    auto robot  = world_->ModelByName(link_full.substr(0, colon));
    auto block  = world_->ModelByName(block_name);
    if (!robot || !block) {
      ROS_WARN("[BlockFollowerPlugin] model 找不到");
      return;
    }
    auto link = robot->GetLink(link_full.substr(colon + 2));
    if (!link) {
      ROS_WARN("[BlockFollowerPlugin] link 找不到: %s", link_full.c_str());
      return;
    }

    ignition::math::Pose3d lp = link->WorldPose();
    ignition::math::Vector3d world_off(bx - lp.Pos().X(),
                                       by - lp.Pos().Y(),
                                       bz - lp.Pos().Z());
    offset_      = lp.Rot().Inverse().RotateVector(world_off);
    target_link_ = link;
    block_model_ = block;
    wake_countdown_ = 0;
    attached_    = true;
    ROS_INFO("[BlockFollowerPlugin] attach OK  offset=(%.3f,%.3f,%.3f)",
             offset_.X(), offset_.Y(), offset_.Z());
  }

  void OnDetach(const std_msgs::String::ConstPtr & /*msg*/) {
    attached_      = false;
    wake_countdown_ = 3000;  // 3000 物理步 ≈ 3 秒，持續喚醒直到落地
    ROS_INFO("[BlockFollowerPlugin] detach，開始持續喚醒");
  }

  void OnUpdate() {
    // ── detach 後持續喚醒：每步呼叫 SetEnabled，對抗 ODE auto-disable ──
    if (wake_countdown_ > 0 && block_model_) {
      wake_countdown_--;
      for (auto &lk : block_model_->GetLinks()) {
        lk->SetEnabled(true);
      }
    }

    if (!attached_ || !target_link_ || !block_model_) return;

    ignition::math::Pose3d lp = target_link_->WorldPose();
    ignition::math::Vector3d new_pos = lp.Pos() + lp.Rot().RotateVector(offset_);
    block_model_->SetWorldPose(
        ignition::math::Pose3d(new_pos, block_model_->WorldPose().Rot()));
  }

  physics::WorldPtr  world_;
  physics::LinkPtr   target_link_;
  physics::ModelPtr  block_model_;
  ignition::math::Vector3d offset_;
  std::atomic<bool> attached_{false};
  std::atomic<int>  wake_countdown_{0};

  event::ConnectionPtr update_conn_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue  cb_queue_;
  ros::Subscriber     attach_sub_, detach_sub_;
  std::thread         ros_thread_;
};

GZ_REGISTER_WORLD_PLUGIN(BlockFollowerPlugin)

}  // namespace gazebo
