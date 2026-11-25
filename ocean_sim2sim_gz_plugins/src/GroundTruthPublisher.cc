// GroundTruthPublisher: ground truth pose and velocity of one link, straight from the physics engine
// (boat motion recordings, ocean_sim2sim_recording; docs/recording.md).
//
// Why not gz::sim::systems::OdometryPublisher (used by the scene for the drone placement): it differentiates the
// published poses for the twist (velocity depends on its publish rate, and a rolling mean smooths it), while this
// plugin reads the link velocities that the Physics system writes on every step (exact, no delay).
//
// Attached to a model; <link_name> = the SDF name of one of its links (here the WAM-V hull link wamv/base_link).
// Every physics step (PostUpdate, not paused) when update_rate <= 0, else at most update_rate per sim second, it
// publishes gz.msgs.Odometry on <topic>:
//   header.stamp           = sim time of the step
//   header frame_id        = <frame_id> (default "world"),  child_frame_id = <link_name>   (header data, as the gz
//                            OdometryPublisher, so ros_gz_bridge fills nav_msgs/Odometry frame_id / child_frame_id)
//   pose                   = link origin pose in the world (position [m], orientation quaternion world -> link)
//   twist.linear           = linear velocity of the link origin [m/s], in the LINK (body) frame
//   twist.angular          = angular velocity [rad/s], in the LINK (body) frame
// World frame of Gazebo = ENU here (world_frame_orientation ENU in the worlds), link frame of a URDF boat = FLU.
//
// SDF:
//   <plugin filename="OceanSim2simGroundTruthPublisher" name="ocean_sim2sim::GroundTruthPublisher">
//     <link_name>wamv/base_link</link_name>
//     <topic>/sim/boat_ground_truth</topic>     (default)
//     <frame_id>world</frame_id>                (default)
//     <update_rate>0</update_rate>              [Hz], 0 = every physics step (default)
//   </plugin>

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace ocean_sim2sim
{
class GroundTruthPublisher : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPreUpdate,
                             public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      gzerr << "[GroundTruthPublisher] must be attached to a model, not loaded\n";
      return;
    }
    if (!_sdf->HasElement("link_name"))
    {
      gzerr << "[GroundTruthPublisher] <link_name> is required, not loaded\n";
      return;
    }
    this->linkName = _sdf->Get<std::string>("link_name");
    this->topic = _sdf->Get<std::string>("topic", "/sim/boat_ground_truth").first;
    this->frameId = _sdf->Get<std::string>("frame_id", "world").first;
    const double rate = _sdf->Get<double>("update_rate", 0.0).first;
    if (rate > 0.0)
      this->period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / rate));
    this->pub = this->node.Advertise<gz::msgs::Odometry>(this->topic);
    if (!this->pub)
    {
      gzerr << "[GroundTruthPublisher] cannot advertise " << this->topic << ", not loaded\n";
      return;
    }
    this->loaded = true;
  }

  // Once (when the link exists): ask the Physics system to write the link velocities every step.
  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->loaded || this->link.has_value())
      return;
    const gz::sim::Entity e = this->model.LinkByName(_ecm, this->linkName);
    if (e == gz::sim::kNullEntity)
    {
      gzerr << "[GroundTruthPublisher] model [" << this->model.Name(_ecm) << "] has no link [" << this->linkName
            << "], publisher off\n";
      this->loaded = false;
      return;
    }
    this->link = gz::sim::Link(e);
    this->link->EnableVelocityChecks(_ecm, true);
    gzmsg << "[GroundTruthPublisher] " << this->model.Name(_ecm) << " / " << this->linkName << " -> " << this->topic
          << (this->period.count() > 0 ? "" : " (every physics step)") << "\n";
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->loaded || !this->link.has_value() || _info.paused)
      return;
    if (this->period.count() > 0)
    {
      if (_info.simTime < this->next)
        return;
      this->next = _info.simTime + this->period;
    }
    const auto pose = this->link->WorldPose(_ecm);
    const auto v = this->link->WorldLinearVelocity(_ecm);
    const auto w = this->link->WorldAngularVelocity(_ecm);
    if (!pose || !v || !w)
      return;                                   // first step after enabling: velocities not written yet
    const gz::math::Quaterniond &q = pose->Rot();
    const gz::math::Vector3d vBody = q.RotateVectorReverse(*v);
    const gz::math::Vector3d wBody = q.RotateVectorReverse(*w);

    gz::msgs::Odometry msg;
    auto *header = msg.mutable_header();
    header->mutable_stamp()->CopyFrom(gz::msgs::Convert(_info.simTime));
    auto *frame = header->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->frameId);
    auto *child = header->add_data();
    child->set_key("child_frame_id");
    child->add_value(this->linkName);
    gz::msgs::Set(msg.mutable_pose(), *pose);
    gz::msgs::Set(msg.mutable_twist()->mutable_linear(), vBody);
    gz::msgs::Set(msg.mutable_twist()->mutable_angular(), wBody);
    this->pub.Publish(msg);
  }

private:
  gz::sim::Model model{gz::sim::kNullEntity};
  std::optional<gz::sim::Link> link;
  std::string linkName, topic, frameId;
  std::chrono::steady_clock::duration period{0};
  std::chrono::steady_clock::duration next{0};
  bool loaded{false};
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
};
}  // namespace ocean_sim2sim

GZ_ADD_PLUGIN(ocean_sim2sim::GroundTruthPublisher, gz::sim::System,
              ocean_sim2sim::GroundTruthPublisher::ISystemConfigure,
              ocean_sim2sim::GroundTruthPublisher::ISystemPreUpdate,
              ocean_sim2sim::GroundTruthPublisher::ISystemPostUpdate)
