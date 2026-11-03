// ContactMonitor: per-link contact forces of a robot arm, input of the protective stop
// (ocean_sim2sim_interface interface_node.py, Isaac Sim reference: data_collection_ur5_gripper
// isaacsim/scripts/protective_stop.py).
//
// Attached to the model that carries the arm (here the WAM-V boat: the arm links are links of the same model).
// The robot links are the links of this model whose SDF name matches <robot_link_regex> (full match). Note that
// URDF -> SDF lumps links joined by fixed joints into their parent: report and match the SDF link names
// (e.g. the gripper base and the wrist camera are collisions of wrist_3_link, the arm base is part of the boat link).
//
// How: the plugin puts a components::ContactSensorData on every collision of the robot links (what the gz Contact
// sensor system does). The Physics system then fills it on every step with the contacts of that collision
// (collision1 = this collision, collision2 = the other one, positions, and wrenches = contact forces).
// Every physics step (PostUpdate, not paused) the plugin adds up per robot link the contact force magnitudes [N],
// separately by what the other collision belongs to:
//   robot : a robot link (self contact, e.g. the fingers touching each other)
//   drone : a link of a model whose name matches <drone_model_regex> (the object to grasp)
//   env   : everything else = the boat (links of this model that are not robot links) and any other model
// and publishes one gz.msgs.StringMsg JSON on <topic> (every step, also when there is no contact, so a reader
// can tell "no contact" from "monitor not running"):
//   {"sec": 12, "nanosec": 345000000, "iter": 2963,
//    "links": {"rh_p12_rn_r2": {"total_N": 3.21, "env_N": 0.0, "robot_N": 0.0, "drone_N": 3.21}, ...}}
//   sec / nanosec = sim time of the step (same as the ROS stamps of /joint_state_broadcaster/joint_states),
//   *_N = sum over the collision pairs (this link's collision, other collision) of that group of |pair force|,
//   pair force = vector sum of the forces at the pair's contact points (exact). total_N = env_N + robot_N + drone_N.
//   Why not the net force vector on the link (Isaac Sim PhysX net contact force): the Physics system gives the SAME
//   wrench to both collisions of a pair (force on whichever shape the engine listed first, DART: not always ours,
//   measured), so the sign per pair is unknown. The sum of pair magnitudes is exact with one touching pair and an
//   upper bound of the net force otherwise (forces from opposite sides do not cancel) = conservative for a stop.
//   Only links with total_N above <min_force> are listed (keeps the message small).
//
// SDF:
//   <plugin filename="OceanSim2simContactMonitor" name="ocean_sim2sim::ContactMonitor">
//     <robot_link_regex>shoulder_link|upper_arm_link|...|rh_p12_rn_[lr][12]</robot_link_regex>
//     <drone_model_regex>drone</drone_model_regex>         (default "drone")
//     <topic>/sim/robot_contacts</topic>                   (default)
//     <min_force>0.01</min_force>                          [N] (default)
//   </plugin>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <regex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/transport/Node.hh>

namespace ocean_sim2sim
{
namespace
{
enum Group { kEnv = 0, kRobot = 1, kDrone = 2, kGroups = 3 };

std::string JsonEscape(const std::string &_s)
{
  std::string out;
  for (char c : _s)
  {
    if (c == '"' || c == '\\')
      out += '\\';
    out += c;
  }
  return out;
}
}  // namespace

class ContactMonitor : public gz::sim::System,
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
      gzerr << "[ContactMonitor] must be attached to a model, not loaded\n";
      return;
    }
    if (!_sdf->HasElement("robot_link_regex"))
    {
      gzerr << "[ContactMonitor] <robot_link_regex> is required, not loaded\n";
      return;
    }
    const std::string robotRe = _sdf->Get<std::string>("robot_link_regex");
    const std::string droneRe = _sdf->Get<std::string>("drone_model_regex", "drone").first;
    try
    {
      this->robotRegex = std::regex(robotRe);
      this->droneRegex = std::regex(droneRe);
    }
    catch (const std::regex_error &e)
    {
      gzerr << "[ContactMonitor] bad regex: " << e.what() << ", not loaded\n";
      return;
    }
    this->topic = _sdf->Get<std::string>("topic", "/sim/robot_contacts").first;
    this->minForce = _sdf->Get<double>("min_force", 0.01).first;
    this->pub = this->node.Advertise<gz::msgs::StringMsg>(this->topic);
    if (!this->pub)
    {
      gzerr << "[ContactMonitor] cannot advertise " << this->topic << ", not loaded\n";
      return;
    }
    this->modelName = this->model.Name(_ecm);
    this->loaded = true;
  }

  // Once (when the model's links exist): contact data components on the robot links' collisions.
  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->loaded || this->ready)
      return;
    std::string names, others;
    for (const auto &linkEntity : this->model.Links(_ecm))
    {
      gz::sim::Link link(linkEntity);
      const std::string name = link.Name(_ecm).value_or("");
      if (!std::regex_match(name, this->robotRegex))
      {
        others += " " + name;
        continue;
      }
      this->robotLinks.insert(linkEntity);
      for (const auto &coll : link.Collisions(_ecm))
      {
        if (!_ecm.Component<gz::sim::components::ContactSensorData>(coll))
          _ecm.CreateComponent(coll, gz::sim::components::ContactSensorData());
        this->collLink[coll] = linkEntity;
        this->linkName[linkEntity] = name;
      }
      names += " " + name;
    }
    if (this->robotLinks.empty())
    {
      gzerr << "[ContactMonitor] no link of model [" << this->modelName << "] matches <robot_link_regex>"
            << " (links:" << others << "), monitor off\n";
      this->loaded = false;
      return;
    }
    gzmsg << "[ContactMonitor] model [" << this->modelName << "] robot links:" << names << " | env (boat) links:"
          << others << " -> " << this->topic << "\n";
    this->ready = true;
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->ready || _info.paused)
      return;
    // per robot link: sum over contact pairs of |pair force| per group [N]
    std::unordered_map<gz::sim::Entity, std::array<double, kGroups>> sums;
    for (const auto &[coll, linkEntity] : this->collLink)
    {
      const auto *data = _ecm.Component<gz::sim::components::ContactSensorData>(coll);
      if (!data)
        continue;
      for (const auto &contact : data->Data().contact())
      {
        // One msgs::Contact = one collision pair, wrench(i) = contact point i. The Physics system copies the SAME
        // wrench to both collisions of a pair (body_1 force = force on the shape that came first in the physics
        // engine's report, which may be either one), so the sign of the force on THIS link is unknown. The points of
        // one pair share the order: |vector sum over the points| is the exact pair force magnitude.
        const gz::sim::Entity other =
            contact.collision1().id() == coll ? contact.collision2().id() : contact.collision1().id();
        gz::math::Vector3d f;
        for (const auto &w : contact.wrench())
        {
          const auto &v = w.body_1_wrench().force();
          f += gz::math::Vector3d(v.x(), v.y(), v.z());
        }
        sums.try_emplace(linkEntity, std::array<double, kGroups>{0.0, 0.0, 0.0});
        sums[linkEntity][this->Classify(other, _ecm)] += f.Length();
      }
    }

    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
    std::string json = "{\"sec\": " + std::to_string(ns / 1000000000) + ", \"nanosec\": " +
                       std::to_string(ns % 1000000000) + ", \"iter\": " + std::to_string(_info.iterations) +
                       ", \"links\": {";
    bool firstLink = true;
    char buf[160];
    for (const auto &[linkEntity, s] : sums)
    {
      const double env = s[kEnv], robot = s[kRobot], drone = s[kDrone];
      const double total = env + robot + drone;
      if (total <= this->minForce)
        continue;
      std::snprintf(buf, sizeof(buf), "{\"total_N\": %.3f, \"env_N\": %.3f, \"robot_N\": %.3f, \"drone_N\": %.3f}",
                    total, env, robot, drone);
      json += std::string(firstLink ? "" : ", ") + "\"" + JsonEscape(this->linkName[linkEntity]) + "\": " + buf;
      firstLink = false;
    }
    json += "}}";
    gz::msgs::StringMsg msg;
    msg.set_data(json);
    this->pub.Publish(msg);
  }

private:
  // Group of the other collision of a contact (cached: entity ids are not reused).
  int Classify(gz::sim::Entity _coll, const gz::sim::EntityComponentManager &_ecm)
  {
    auto it = this->groupCache.find(_coll);
    if (it != this->groupCache.end())
      return it->second;
    int group = kEnv;
    const auto *parent = _ecm.Component<gz::sim::components::ParentEntity>(_coll);
    if (parent)
    {
      const gz::sim::Entity link = parent->Data();
      if (this->robotLinks.count(link))
        group = kRobot;
      else
      {
        const gz::sim::Entity top = gz::sim::topLevelModel(link, _ecm);
        if (top != this->model.Entity())
        {
          const auto *name = _ecm.Component<gz::sim::components::Name>(top);
          if (name && std::regex_match(name->Data(), this->droneRegex))
            group = kDrone;
        }
      }
    }
    this->groupCache[_coll] = group;
    return group;
  }

  gz::sim::Model model{gz::sim::kNullEntity};
  std::string modelName;
  std::regex robotRegex, droneRegex;
  std::string topic;
  double minForce{0.01};
  bool loaded{false}, ready{false};
  std::unordered_set<gz::sim::Entity> robotLinks;
  std::unordered_map<gz::sim::Entity, gz::sim::Entity> collLink;   // robot collision -> its link
  std::unordered_map<gz::sim::Entity, std::string> linkName;
  std::unordered_map<gz::sim::Entity, int> groupCache;
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
};
}  // namespace ocean_sim2sim

GZ_ADD_PLUGIN(ocean_sim2sim::ContactMonitor, gz::sim::System, ocean_sim2sim::ContactMonitor::ISystemConfigure,
              ocean_sim2sim::ContactMonitor::ISystemPreUpdate, ocean_sim2sim::ContactMonitor::ISystemPostUpdate)
