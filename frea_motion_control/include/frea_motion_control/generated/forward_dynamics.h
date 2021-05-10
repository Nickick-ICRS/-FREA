#ifndef IIT_ROBOT_FREA_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_FREA_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/robcogen_commons.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace iit {
namespace frea {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot frea.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */

namespace tpl {

template <typename TRAIT>
class ForwardDynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Convenient type aliases:

    typedef typename TRAIT::Scalar Scalar;

    typedef iit::rbd::Core<Scalar> CoreS;

    typedef typename CoreS::ForceVector Force;
    typedef typename CoreS::VelocityVector Velocity;
    typedef typename CoreS::VelocityVector Acceleration;
    typedef typename CoreS::Column6D Column6DS;
    typedef typename CoreS::Matrix66 Matrix66S;
    typedef LinkDataMap<Force> ExtForces;
    typedef typename iit::frea::tpl::JointState<Scalar> JointState;
    typedef iit::frea::tpl::MotionTransforms<TRAIT> MTransforms;

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot frea, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties<TRAIT>& in, MTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
        JointState& qdd, // output parameter
        const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, // output parameter
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties<TRAIT>* inertiaProps;
    MTransforms* motionTransforms;

    Matrix66S vcross; // support variable
    Matrix66S Ia_r;   // support variable, articulated inertia in the case of a revolute joint

    // Link 'chassis_link' :
    Matrix66S chassis_link_AI;
    Velocity chassis_link_a;
    Velocity chassis_link_v;
    Velocity chassis_link_c;
    Force    chassis_link_p;

    Column6DS chassis_link_U;
    Scalar chassis_link_D;
    Scalar chassis_link_u;
    // Link 'left_wheel_link' :
    Matrix66S left_wheel_link_AI;
    Velocity left_wheel_link_a;
    Velocity left_wheel_link_v;
    Velocity left_wheel_link_c;
    Force    left_wheel_link_p;

    Column6DS left_wheel_link_U;
    Scalar left_wheel_link_D;
    Scalar left_wheel_link_u;
    // Link 'right_wheel_link' :
    Matrix66S right_wheel_link_AI;
    Velocity right_wheel_link_a;
    Velocity right_wheel_link_v;
    Velocity right_wheel_link_c;
    Force    right_wheel_link_p;

    Column6DS right_wheel_link_U;
    Scalar right_wheel_link_D;
    Scalar right_wheel_link_u;
    // Link 'lower_neck_link' :
    Matrix66S lower_neck_link_AI;
    Velocity lower_neck_link_a;
    Velocity lower_neck_link_v;
    Velocity lower_neck_link_c;
    Force    lower_neck_link_p;

    Column6DS lower_neck_link_U;
    Scalar lower_neck_link_D;
    Scalar lower_neck_link_u;
    // Link 'upper_neck_link' :
    Matrix66S upper_neck_link_AI;
    Velocity upper_neck_link_a;
    Velocity upper_neck_link_v;
    Velocity upper_neck_link_c;
    Force    upper_neck_link_p;

    Column6DS upper_neck_link_U;
    Scalar upper_neck_link_D;
    Scalar upper_neck_link_u;
    // Link 'head_link' :
    Matrix66S head_link_AI;
    Velocity head_link_a;
    Velocity head_link_v;
    Velocity head_link_c;
    Force    head_link_p;

    Column6DS head_link_U;
    Scalar head_link_D;
    Scalar head_link_u;
    // Link 'mouth_link' :
    Matrix66S mouth_link_AI;
    Velocity mouth_link_a;
    Velocity mouth_link_v;
    Velocity mouth_link_c;
    Force    mouth_link_p;

    Column6DS mouth_link_U;
    Scalar mouth_link_D;
    Scalar mouth_link_u;
    // Link 'left_ear_link' :
    Matrix66S left_ear_link_AI;
    Velocity left_ear_link_a;
    Velocity left_ear_link_v;
    Velocity left_ear_link_c;
    Force    left_ear_link_p;

    Column6DS left_ear_link_U;
    Scalar left_ear_link_D;
    Scalar left_ear_link_u;
    // Link 'right_ear_link' :
    Matrix66S right_ear_link_AI;
    Velocity right_ear_link_a;
    Velocity right_ear_link_v;
    Velocity right_ear_link_c;
    Force    right_ear_link_p;

    Column6DS right_ear_link_U;
    Scalar right_ear_link_D;
    Scalar right_ear_link_u;
    // Link 'upper_tail_link' :
    Matrix66S upper_tail_link_AI;
    Velocity upper_tail_link_a;
    Velocity upper_tail_link_v;
    Velocity upper_tail_link_c;
    Force    upper_tail_link_p;

    Column6DS upper_tail_link_U;
    Scalar upper_tail_link_D;
    Scalar upper_tail_link_u;
    // Link 'lower_tail_link' :
    Matrix66S lower_tail_link_AI;
    Velocity lower_tail_link_a;
    Velocity lower_tail_link_v;
    Velocity lower_tail_link_c;
    Force    lower_tail_link_p;

    Column6DS lower_tail_link_U;
    Scalar lower_tail_link_D;
    Scalar lower_tail_link_u;
private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_chassis_link_X_fr_world)(q);
    (motionTransforms-> fr_left_wheel_link_X_fr_chassis_link)(q);
    (motionTransforms-> fr_right_wheel_link_X_fr_chassis_link)(q);
    (motionTransforms-> fr_lower_neck_link_X_fr_chassis_link)(q);
    (motionTransforms-> fr_upper_neck_link_X_fr_lower_neck_link)(q);
    (motionTransforms-> fr_head_link_X_fr_upper_neck_link)(q);
    (motionTransforms-> fr_mouth_link_X_fr_head_link)(q);
    (motionTransforms-> fr_left_ear_link_X_fr_head_link)(q);
    (motionTransforms-> fr_right_ear_link_X_fr_head_link)(q);
    (motionTransforms-> fr_upper_tail_link_X_fr_chassis_link)(q);
    (motionTransforms-> fr_lower_tail_link_X_fr_upper_tail_link)(q);
}

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, qd, tau, fext);
}

}

typedef tpl::ForwardDynamics<iit::rbd::DoubleTrait> ForwardDynamics;

}
}
}

#include "forward_dynamics.impl.h"

#endif
