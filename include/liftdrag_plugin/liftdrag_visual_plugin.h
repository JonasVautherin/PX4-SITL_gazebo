#ifndef _GAZEBO_LIFT_DRAG_VISUAL_PLUGIN_HH_
#define _GAZEBO_LIFT_DRAG_VISUAL_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/TransportTypes.hh>

#include "Force.pb.h"

#define FORCE_SCALE 1//8E-2 // scale between the forces intensities and the vectors length (unity N^-1)
#define ARROW_LENGTH 1//.05

namespace gazebo
{
  typedef const boost::shared_ptr<const physics_msgs::msgs::Force> ConstForcePtr;

  /// \brief A visual plugin that draws the liftdrag force
  class GAZEBO_VISIBLE LiftDragVisualPlugin : public VisualPlugin
  {
    /// \brief Constructor.
    public: LiftDragVisualPlugin();

    /// \brief Destructor.
    public: ~LiftDragVisualPlugin();

    // Documentation Inherited.
    public: virtual void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Init();

    /// \brief Pointer to visual containing plugin.
    protected: rendering::VisualPtr visual;

    /// \brief Pointer to element containing sdf.
    protected: sdf::ElementPtr sdf;

    private: void OnUpdate(ConstForcePtr& force_msg);
    private: void UpdateVector(const ignition::math::Vector3d& center, const ignition::math::Vector3d& force);

    private: transport::SubscriberPtr subs;
    private: transport::NodePtr node;
    private: rendering::DynamicLinesPtr forceVector;
  };
}

#endif
