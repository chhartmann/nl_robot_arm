#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <gz/common/Console.hh>
#include <gz/msgs/contacts.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Static.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace nlra
{
class GraspFix final : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate,
                       public gz::sim::ISystemPostUpdate
{
  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      gzerr << "GraspFix must be attached to a model.\n";
      return;
    }

    this->palmLinkName = _sdf->Get<std::string>("palm_link");
    this->leftLinkName = _sdf->Get<std::string>("left_finger_link");
    this->rightLinkName = _sdf->Get<std::string>("right_finger_link");
    this->leftSensorName = _sdf->Get<std::string>("left_contact_sensor");
    this->rightSensorName = _sdf->Get<std::string>("right_contact_sensor");
    this->gripperJointName = _sdf->Get<std::string>(
        "gripper_joint", this->gripperJointName).first;
    this->gripperClosedAngle = _sdf->Get<double>(
        "gripper_closed_angle", this->gripperClosedAngle).first;
    this->gripperOpenAngle = _sdf->Get<double>(
        "gripper_open_angle", this->gripperOpenAngle).first;
    this->gripCountThreshold = _sdf->Get<unsigned int>(
        "grip_count_threshold", this->gripCountThreshold).first;
    this->releaseCountThreshold = _sdf->Get<unsigned int>(
        "release_count_threshold", this->releaseCountThreshold).first;
    this->configured = true;
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->configured)
      return;

    if (!this->initialized)
    {
      this->initialized = this->ResolveEntities(_ecm);
      if (!this->initialized)
        return;
      gzmsg << "GraspFix ready for model [" << this->model.Name(_ecm) << "]\n";
    }

    if (this->detachRequested &&
        this->detachableJoint != gz::sim::kNullEntity)
    {
      _ecm.RequestRemoveEntity(this->detachableJoint);
      gzmsg << "GraspFix detached object model [" << this->attachedModel << "]\n";
      this->detachableJoint = gz::sim::kNullEntity;
      this->attachedModel = gz::sim::kNullEntity;
      this->releaseCount = 0;
      this->detachRequested = false;
      this->gripCandidate = gz::sim::kNullEntity;
      this->gripCount = 0;
    }

    if (this->attachRequested != gz::sim::kNullEntity &&
        this->detachableJoint == gz::sim::kNullEntity)
    {
      this->detachableJoint = _ecm.CreateEntity();
      _ecm.CreateComponent(this->detachableJoint,
          gz::sim::components::DetachableJoint(
              {this->palmLink, this->attachRequested, "fixed"}));
      this->attachedModel = _ecm.ParentEntity(this->attachRequested);
      gzmsg << "GraspFix attached object model [" << this->attachedModel << "]\n";
      this->attachRequested = gz::sim::kNullEntity;
      this->gripCount = 0;
    }
  }

  public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                          const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->initialized)
      return;

    gz::msgs::Contacts leftContacts;
    gz::msgs::Contacts rightContacts;
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(this->contactMutex);
      if (now - this->leftContactTime < this->contactTimeout)
        leftContacts = this->leftContacts;
      if (now - this->rightContactTime < this->contactTimeout)
        rightContacts = this->rightContacts;
    }

    const auto leftObjects = this->TouchedObjects(leftContacts, _ecm);
    const auto rightObjects = this->TouchedObjects(rightContacts, _ecm);
    gz::sim::Entity candidateModel = gz::sim::kNullEntity;
    gz::sim::Entity candidateLink = gz::sim::kNullEntity;

    if (leftObjects.size() == 1 && rightObjects.size() == 1)
    {
      const auto &[leftModel, leftLink] = *leftObjects.begin();
      const auto &[rightModel, rightLink] = *rightObjects.begin();
      if (leftModel == rightModel)
      {
        candidateModel = leftModel;
        candidateLink = leftLink;
      }
    }

    // A curved or compliant object can contact only one fingertip at the
    // nominal target angle. Once the gripper is actually closing, a
    // persistent one-sided contact is sufficient for the simulation grasp
    // fix. Requiring both sensors makes cylinders and edge contacts fail.
    if (candidateModel == gz::sim::kNullEntity &&
        this->GripperIsClosed(_ecm))
    {
      if (leftObjects.size() == 1 && rightObjects.empty())
      {
        candidateModel = leftObjects.begin()->first;
        candidateLink = leftObjects.begin()->second;
      }
      else if (rightObjects.size() == 1 && leftObjects.empty())
      {
        candidateModel = rightObjects.begin()->first;
        candidateLink = rightObjects.begin()->second;
      }
    }

    if (this->detachableJoint != gz::sim::kNullEntity)
    {
      // Contact manifolds can flicker while the arm transfers an object.
      // Only an opened gripper is an intentional release; losing contact
      // while closed must not remove the fixed joint.
      if (this->GripperIsOpen(_ecm))
      {
        if (++this->releaseCount >= this->releaseCountThreshold)
          this->detachRequested = true;
      }
      else
      {
        this->releaseCount = 0;
      }
      return;
    }

    if (candidateModel != gz::sim::kNullEntity &&
        candidateModel == this->gripCandidate)
      ++this->gripCount;
    else
    {
      this->gripCandidate = candidateModel;
      this->gripCount = candidateModel == gz::sim::kNullEntity ? 0 : 1;
    }

    if (this->gripCount >= this->gripCountThreshold)
      this->attachRequested = candidateLink;
  }

  private: bool ResolveEntities(gz::sim::EntityComponentManager &_ecm)
  {
    this->palmLink = this->LinkByName(this->palmLinkName, _ecm);
    this->leftLink = this->LinkByName(this->leftLinkName, _ecm);
    this->rightLink = this->LinkByName(this->rightLinkName, _ecm);
    this->gripperJoint = _ecm.EntityByComponents(
        gz::sim::components::Joint(),
        gz::sim::components::Name(this->gripperJointName));
    if (this->palmLink == gz::sim::kNullEntity ||
        this->leftLink == gz::sim::kNullEntity ||
        this->rightLink == gz::sim::kNullEntity ||
        this->gripperJoint == gz::sim::kNullEntity)
    {
      if (this->gripperJoint == gz::sim::kNullEntity)
        gzerr << "GraspFix could not resolve gripper joint ["
              << this->gripperJointName << "].\n";
      return false;
    }

    // Ask the physics system to publish the knuckle position into the ECM so
    // GripperIsClosed/GripperIsOpen can observe the actual close/open state.
    // Physics only fills JointPosition for joints that already have the
    // component, so create it here with a neutral initial value.
    if (_ecm.Component<gz::sim::components::JointPosition>(
            this->gripperJoint) == nullptr)
    {
      _ecm.CreateComponent(this->gripperJoint,
          gz::sim::components::JointPosition({0.0}));
    }

    const auto leftSensor = _ecm.EntityByComponents(
        gz::sim::components::Name(this->leftSensorName),
        gz::sim::components::ParentEntity(this->leftLink));
    const auto rightSensor = _ecm.EntityByComponents(
        gz::sim::components::Name(this->rightSensorName),
        gz::sim::components::ParentEntity(this->rightLink));
    if (leftSensor == gz::sim::kNullEntity || rightSensor == gz::sim::kNullEntity)
      return false;

    const auto leftTopic = gz::sim::scopedName(leftSensor, _ecm, "/") + "/contact";
    const auto rightTopic = gz::sim::scopedName(rightSensor, _ecm, "/") + "/contact";
    const bool leftSubscribed = this->node.Subscribe(leftTopic,
        std::function<void(const gz::msgs::Contacts &)>(
            [this](const gz::msgs::Contacts &_msg) { this->OnLeftContact(_msg); }));
    const bool rightSubscribed = this->node.Subscribe(rightTopic,
        std::function<void(const gz::msgs::Contacts &)>(
            [this](const gz::msgs::Contacts &_msg) { this->OnRightContact(_msg); }));
    if (!leftSubscribed || !rightSubscribed)
    {
      gzerr << "GraspFix could not subscribe to fingertip contact topics.\n";
      return false;
    }
    return true;
  }

  private: gz::sim::Entity LinkByName(
      const std::string &_name, const gz::sim::EntityComponentManager &_ecm) const
  {
    const auto modelLink = this->model.LinkByName(_ecm, _name);
    return modelLink != gz::sim::kNullEntity ? modelLink :
        _ecm.EntityByComponents(gz::sim::components::Link(),
            gz::sim::components::Name(_name));
  }

  private: bool GripperIsClosed(
      const gz::sim::EntityComponentManager &_ecm) const
  {
    const auto *position = _ecm.Component<gz::sim::components::JointPosition>(
        this->gripperJoint);
    return position != nullptr && !position->Data().empty() &&
        position->Data()[0] >= this->gripperClosedAngle;
  }

  private: bool GripperIsOpen(
      const gz::sim::EntityComponentManager &_ecm) const
  {
    const auto *position = _ecm.Component<gz::sim::components::JointPosition>(
        this->gripperJoint);
    return position != nullptr && !position->Data().empty() &&
        position->Data()[0] <= this->gripperOpenAngle;
  }

  private: std::unordered_map<gz::sim::Entity, gz::sim::Entity> TouchedObjects(
      const gz::msgs::Contacts &_contacts,
      const gz::sim::EntityComponentManager &_ecm) const
  {
    std::unordered_map<gz::sim::Entity, gz::sim::Entity> objects;
    for (const auto &contact : _contacts.contact())
    {
      const auto firstCollision =
          static_cast<gz::sim::Entity>(contact.collision1().id());
      const auto secondCollision =
          static_cast<gz::sim::Entity>(contact.collision2().id());
      const auto firstLink = _ecm.ParentEntity(firstCollision);
      const auto secondLink = _ecm.ParentEntity(secondCollision);
      const auto firstModel = _ecm.ParentEntity(firstLink);
      const auto secondModel = _ecm.ParentEntity(secondLink);
      const auto objectLink = firstModel == this->model.Entity() ? secondLink :
          (secondModel == this->model.Entity() ? firstLink : gz::sim::kNullEntity);
      const auto objectModel = _ecm.ParentEntity(objectLink);
      if (objectLink == gz::sim::kNullEntity ||
          objectModel == gz::sim::kNullEntity)
        continue;

      const auto *isStatic = _ecm.Component<gz::sim::components::Static>(objectModel);
      if (isStatic == nullptr || !isStatic->Data())
        objects.emplace(objectModel, objectLink);
    }
    return objects;
  }

  private: void OnLeftContact(const gz::msgs::Contacts &_msg)
  {
    std::lock_guard<std::mutex> lock(this->contactMutex);
    this->leftContacts = _msg;
    this->leftContactTime = std::chrono::steady_clock::now();
  }

  private: void OnRightContact(const gz::msgs::Contacts &_msg)
  {
    std::lock_guard<std::mutex> lock(this->contactMutex);
    this->rightContacts = _msg;
    this->rightContactTime = std::chrono::steady_clock::now();
  }

  private: gz::sim::Model model{gz::sim::kNullEntity};
  private: gz::sim::Entity palmLink{gz::sim::kNullEntity};
  private: gz::sim::Entity leftLink{gz::sim::kNullEntity};
  private: gz::sim::Entity rightLink{gz::sim::kNullEntity};
  private: gz::sim::Entity gripperJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity detachableJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity attachedModel{gz::sim::kNullEntity};
  private: gz::sim::Entity gripCandidate{gz::sim::kNullEntity};
  private: gz::sim::Entity attachRequested{gz::sim::kNullEntity};
  private: std::string palmLinkName;
  private: std::string leftLinkName;
  private: std::string rightLinkName;
  private: std::string leftSensorName;
  private: std::string rightSensorName;
  private: std::string gripperJointName{"robotiq_85_left_knuckle_joint"};
  private: gz::transport::Node node;
  private: std::mutex contactMutex;
  private: gz::msgs::Contacts leftContacts;
  private: gz::msgs::Contacts rightContacts;
  private: std::chrono::steady_clock::time_point leftContactTime{};
  private: std::chrono::steady_clock::time_point rightContactTime{};
  private: std::chrono::milliseconds contactTimeout{50};
  private: unsigned int gripCountThreshold{10};
  private: unsigned int releaseCountThreshold{20};
  private: double gripperClosedAngle{0.20};
  private: double gripperOpenAngle{0.08};
  private: unsigned int gripCount{0};
  private: unsigned int releaseCount{0};
  private: bool detachRequested{false};
  private: bool configured{false};
  private: bool initialized{false};
};
}  // namespace nlra

GZ_ADD_PLUGIN(nlra::GraspFix,
              gz::sim::System,
              nlra::GraspFix::ISystemConfigure,
              nlra::GraspFix::ISystemPreUpdate,
              nlra::GraspFix::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(nlra::GraspFix, "nlra::systems::GraspFix")
