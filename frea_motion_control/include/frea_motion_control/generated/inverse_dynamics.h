#ifndef IIT_FREA_INVERSE_DYNAMICS_H_
#define IIT_FREA_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/robcogen_commons.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace iit {
namespace frea {
namespace dyn {

/**
 * The Inverse Dynamics routine for the robot frea.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */

namespace tpl {

template <typename TRAIT>
class InverseDynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef typename TRAIT::Scalar Scalar;

    typedef iit::rbd::Core<Scalar> CoreS;

    typedef typename CoreS::ForceVector Force;
    typedef typename CoreS::VelocityVector Velocity;
    typedef typename CoreS::VelocityVector Acceleration;
    typedef typename CoreS::Matrix66 Matrix66s;
    typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> InertiaMatrix;
    typedef iit::frea::tpl::JointState<Scalar> JointState;
    typedef LinkDataMap<Force> ExtForces;
    typedef iit::frea::tpl::MotionTransforms<TRAIT> MTransforms;
    typedef InertiaProperties<TRAIT> IProperties;

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot frea, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(IProperties& in, MTransforms& tr);

    /** \name Inverse dynamics
     * The full Newton-Euler algorithm for the inverse dynamics of this robot.
     *
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */
    ///@{
    void id(
        JointState& jForces,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}

    /** \name Gravity terms
     * The joint forces (linear or rotational) required to compensate
     * for the effect of gravity, in a specific configuration.
     */
    ///@{
    void G_terms(JointState& jForces, const JointState& q);
    void G_terms(JointState& jForces);
    ///@}

    /** \name Centrifugal and Coriolis terms
     * The forces (linear or rotational) acting on the joints due to centrifugal and
     * Coriolis effects, for a specific configuration.
     */
    ///@{
    void C_terms(JointState& jForces, const JointState& q, const JointState& qd);
    void C_terms(JointState& jForces, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Velocity& getVelocity_chassis_link() const { return chassis_link_v; }
    const Acceleration& getAcceleration_chassis_link() const { return chassis_link_a; }
    const Force& getForce_chassis_link() const { return chassis_link_f; }
    const Velocity& getVelocity_left_wheel_link() const { return left_wheel_link_v; }
    const Acceleration& getAcceleration_left_wheel_link() const { return left_wheel_link_a; }
    const Force& getForce_left_wheel_link() const { return left_wheel_link_f; }
    const Velocity& getVelocity_right_wheel_link() const { return right_wheel_link_v; }
    const Acceleration& getAcceleration_right_wheel_link() const { return right_wheel_link_a; }
    const Force& getForce_right_wheel_link() const { return right_wheel_link_f; }
    const Velocity& getVelocity_lower_neck_link() const { return lower_neck_link_v; }
    const Acceleration& getAcceleration_lower_neck_link() const { return lower_neck_link_a; }
    const Force& getForce_lower_neck_link() const { return lower_neck_link_f; }
    const Velocity& getVelocity_upper_neck_link() const { return upper_neck_link_v; }
    const Acceleration& getAcceleration_upper_neck_link() const { return upper_neck_link_a; }
    const Force& getForce_upper_neck_link() const { return upper_neck_link_f; }
    const Velocity& getVelocity_head_link() const { return head_link_v; }
    const Acceleration& getAcceleration_head_link() const { return head_link_a; }
    const Force& getForce_head_link() const { return head_link_f; }
    const Velocity& getVelocity_mouth_link() const { return mouth_link_v; }
    const Acceleration& getAcceleration_mouth_link() const { return mouth_link_a; }
    const Force& getForce_mouth_link() const { return mouth_link_f; }
    const Velocity& getVelocity_left_ear_link() const { return left_ear_link_v; }
    const Acceleration& getAcceleration_left_ear_link() const { return left_ear_link_a; }
    const Force& getForce_left_ear_link() const { return left_ear_link_f; }
    const Velocity& getVelocity_right_ear_link() const { return right_ear_link_v; }
    const Acceleration& getAcceleration_right_ear_link() const { return right_ear_link_a; }
    const Force& getForce_right_ear_link() const { return right_ear_link_f; }
    const Velocity& getVelocity_upper_tail_link() const { return upper_tail_link_v; }
    const Acceleration& getAcceleration_upper_tail_link() const { return upper_tail_link_a; }
    const Force& getForce_upper_tail_link() const { return upper_tail_link_f; }
    const Velocity& getVelocity_lower_tail_link() const { return lower_tail_link_v; }
    const Acceleration& getAcceleration_lower_tail_link() const { return lower_tail_link_a; }
    const Force& getForce_lower_tail_link() const { return lower_tail_link_f; }
    ///@}
protected:
    void firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext);
    void secondPass(JointState& jForces);

private:
    IProperties* inertiaProps;
    MTransforms* xm;
private:
    Matrix66s vcross; // support variable
    // Link 'chassis_link' :
    const InertiaMatrix& chassis_link_I;
    Velocity      chassis_link_v;
    Acceleration  chassis_link_a;
    Force         chassis_link_f;
    // Link 'left_wheel_link' :
    const InertiaMatrix& left_wheel_link_I;
    Velocity      left_wheel_link_v;
    Acceleration  left_wheel_link_a;
    Force         left_wheel_link_f;
    // Link 'right_wheel_link' :
    const InertiaMatrix& right_wheel_link_I;
    Velocity      right_wheel_link_v;
    Acceleration  right_wheel_link_a;
    Force         right_wheel_link_f;
    // Link 'lower_neck_link' :
    const InertiaMatrix& lower_neck_link_I;
    Velocity      lower_neck_link_v;
    Acceleration  lower_neck_link_a;
    Force         lower_neck_link_f;
    // Link 'upper_neck_link' :
    const InertiaMatrix& upper_neck_link_I;
    Velocity      upper_neck_link_v;
    Acceleration  upper_neck_link_a;
    Force         upper_neck_link_f;
    // Link 'head_link' :
    const InertiaMatrix& head_link_I;
    Velocity      head_link_v;
    Acceleration  head_link_a;
    Force         head_link_f;
    // Link 'mouth_link' :
    const InertiaMatrix& mouth_link_I;
    Velocity      mouth_link_v;
    Acceleration  mouth_link_a;
    Force         mouth_link_f;
    // Link 'left_ear_link' :
    const InertiaMatrix& left_ear_link_I;
    Velocity      left_ear_link_v;
    Acceleration  left_ear_link_a;
    Force         left_ear_link_f;
    // Link 'right_ear_link' :
    const InertiaMatrix& right_ear_link_I;
    Velocity      right_ear_link_v;
    Acceleration  right_ear_link_a;
    Force         right_ear_link_f;
    // Link 'upper_tail_link' :
    const InertiaMatrix& upper_tail_link_I;
    Velocity      upper_tail_link_v;
    Acceleration  upper_tail_link_a;
    Force         upper_tail_link_f;
    // Link 'lower_tail_link' :
    const InertiaMatrix& lower_tail_link_I;
    Velocity      lower_tail_link_v;
    Acceleration  lower_tail_link_a;
    Force         lower_tail_link_f;


private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::setJointStatus(const JointState& q) const
{
    (xm->fr_chassis_link_X_fr_world)(q);
    (xm->fr_left_wheel_link_X_fr_chassis_link)(q);
    (xm->fr_right_wheel_link_X_fr_chassis_link)(q);
    (xm->fr_lower_neck_link_X_fr_chassis_link)(q);
    (xm->fr_upper_neck_link_X_fr_lower_neck_link)(q);
    (xm->fr_head_link_X_fr_upper_neck_link)(q);
    (xm->fr_mouth_link_X_fr_head_link)(q);
    (xm->fr_left_ear_link_X_fr_head_link)(q);
    (xm->fr_right_ear_link_X_fr_head_link)(q);
    (xm->fr_upper_tail_link_X_fr_chassis_link)(q);
    (xm->fr_lower_tail_link_X_fr_upper_tail_link)(q);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::G_terms(JointState& jForces, const JointState& q)
{
    setJointStatus(q);
    G_terms(jForces);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::C_terms(JointState& jForces, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms(jForces, qd);
}

template <typename TRAIT>
inline void InverseDynamics<TRAIT>::id(
    JointState& jForces,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, qd, qdd, fext);
}

}

typedef tpl::InverseDynamics<rbd::DoubleTrait> InverseDynamics;

}
}

}

#include "inverse_dynamics.impl.h"

#endif
