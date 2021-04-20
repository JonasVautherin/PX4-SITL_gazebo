#include "liftdrag_plugin/liftdrag_visual_plugin.h"

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(LiftDragVisualPlugin)

LiftDragVisualPlugin::LiftDragVisualPlugin() {}

LiftDragVisualPlugin::~LiftDragVisualPlugin() {}

void LiftDragVisualPlugin::Load(rendering::VisualPtr _visual,
                                sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_visual, "LiftDragVisualPlugin _visual pointer is NULL");
  GZ_ASSERT(_sdf, "LiftDragVisualPlugin _sdf pointer is NULL");

  this->visual = _visual;
  this->sdf = _sdf;

  gzdbg << "My first visual plugin" << std::endl;
}

void LiftDragVisualPlugin::Init()
{
  this->node.reset(new gazebo::transport::Node());
  this->node->Init();
  this->subs.reset();

  if (this->sdf->HasElement("topic_name")) {
      const auto lift_force_topic = this->sdf->Get<std::string>("topic_name");
      this->subs = this->node->Subscribe("~/" + lift_force_topic, &LiftDragVisualPlugin::OnUpdate, this);
      gzdbg << "Subscribing on ~/" << lift_force_topic << std::endl;
  }

  auto forceVector = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  this->forceVector.reset(forceVector);
  this->forceVector->setMaterial("Gazebo/Blue");
  this->forceVector->setVisibilityFlags(GZ_VISIBILITY_GUI);

  if (this->sdf->HasElement("material")) {
    this->forceVector->setMaterial(this->sdf->Get<std::string>("material"));
  }

  for(int k = 0; k < 6; ++k) { // -> needs three lines, so 6 points
    this->forceVector->AddPoint(ignition::math::Vector3d::Zero);
  }

  this->forceVector->Update();
}

void LiftDragVisualPlugin::OnUpdate(ConstForcePtr& force_msg)
{
  ignition::math::Vector3d force;
  auto force_vector_msg = force_msg->force();
  force.Set(force_vector_msg.x(), force_vector_msg.y(), force_vector_msg.z());
  //gzdbg << "liftdrag force: " << force << std::endl;

  ignition::math::Vector3d center;
  auto force_center_msg = force_msg->center();
  center.Set(force_center_msg.x(), force_center_msg.y(), force_center_msg.z());

  this->UpdateVector(center, force);
}

void LiftDragVisualPlugin::UpdateVector(const ignition::math::Vector3d& center, const ignition::math::Vector3d& force)
{
  ignition::math::Vector3d begin = center;
  ignition::math::Vector3d end = force; //begin + FORCE_SCALE * (this->visual->WorldPose().Rot().RotateVector(force));

  // draw a cute arrow, just as a vector should be represented
  this->forceVector->SetPoint(0, begin);
  this->forceVector->SetPoint(1, end);
  this->forceVector->SetPoint(2, end);
  this->forceVector->SetPoint(3, end - ARROW_LENGTH * ignition::math::Matrix3d(1, 0, 0, 0, 0.9848, -0.1736, 0,  0.1736, 0.9848)*(end - begin).Normalize());
  this->forceVector->SetPoint(4, end);
  this->forceVector->SetPoint(5, end - ARROW_LENGTH * ignition::math::Matrix3d(1, 0, 0, 0, 0.9848,  0.1736, 0, -0.1736, 0.9848)*(end - begin).Normalize());
}
