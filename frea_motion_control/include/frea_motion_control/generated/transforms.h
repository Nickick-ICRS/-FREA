#ifndef FREA_TRANSFORMS_H_
#define FREA_TRANSFORMS_H_

#include <Eigen/Dense>
#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include <iit/rbd/traits/DoubleTrait.h>
#include "kinematics_parameters.h"

namespace iit {
namespace frea {

template<typename SCALAR, class M>
class TransformMotion : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformForce : public iit::rbd::SpatialTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename SCALAR, class M>
class TransformHomogeneous : public iit::rbd::HomogeneousTransformBase<tpl::JointState<SCALAR>, M> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

namespace tpl {


/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
template <typename TRAIT>
class MotionTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformMotion<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_world_X_fr_chassis_link : public TransformMotion<Scalar, Type_fr_world_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_chassis_link();
        const Type_fr_world_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_world : public TransformMotion<Scalar, Type_fr_chassis_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_world();
        const Type_fr_chassis_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_head_link : public TransformMotion<Scalar, Type_fr_upper_neck_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_head_link();
        const Type_fr_upper_neck_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_upper_neck_link : public TransformMotion<Scalar, Type_fr_head_link_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_upper_neck_link();
        const Type_fr_head_link_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_left_ear_link : public TransformMotion<Scalar, Type_fr_head_link_X_fr_left_ear_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_left_ear_link();
        const Type_fr_head_link_X_fr_left_ear_link& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_head_link : public TransformMotion<Scalar, Type_fr_left_ear_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_head_link();
        const Type_fr_left_ear_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_left_wheel_link : public TransformMotion<Scalar, Type_fr_chassis_link_X_fr_left_wheel_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_left_wheel_link();
        const Type_fr_chassis_link_X_fr_left_wheel_link& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_X_fr_chassis_link : public TransformMotion<Scalar, Type_fr_left_wheel_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_X_fr_chassis_link();
        const Type_fr_left_wheel_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_lower_neck_link : public TransformMotion<Scalar, Type_fr_chassis_link_X_fr_lower_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_lower_neck_link();
        const Type_fr_chassis_link_X_fr_lower_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_chassis_link : public TransformMotion<Scalar, Type_fr_lower_neck_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_chassis_link();
        const Type_fr_lower_neck_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_lower_tail_link : public TransformMotion<Scalar, Type_fr_upper_tail_link_X_fr_lower_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_lower_tail_link();
        const Type_fr_upper_tail_link_X_fr_lower_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_upper_tail_link : public TransformMotion<Scalar, Type_fr_lower_tail_link_X_fr_upper_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_upper_tail_link();
        const Type_fr_lower_tail_link_X_fr_upper_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_mouth_link : public TransformMotion<Scalar, Type_fr_head_link_X_fr_mouth_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_mouth_link();
        const Type_fr_head_link_X_fr_mouth_link& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_head_link : public TransformMotion<Scalar, Type_fr_mouth_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_head_link();
        const Type_fr_mouth_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_right_ear_link : public TransformMotion<Scalar, Type_fr_head_link_X_fr_right_ear_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_right_ear_link();
        const Type_fr_head_link_X_fr_right_ear_link& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_head_link : public TransformMotion<Scalar, Type_fr_right_ear_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_head_link();
        const Type_fr_right_ear_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_right_wheel_link : public TransformMotion<Scalar, Type_fr_chassis_link_X_fr_right_wheel_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_right_wheel_link();
        const Type_fr_chassis_link_X_fr_right_wheel_link& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_X_fr_chassis_link : public TransformMotion<Scalar, Type_fr_right_wheel_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_X_fr_chassis_link();
        const Type_fr_right_wheel_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_upper_neck_link : public TransformMotion<Scalar, Type_fr_chassis_link_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_upper_neck_link();
        const Type_fr_chassis_link_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_chassis_link : public TransformMotion<Scalar, Type_fr_upper_neck_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_chassis_link();
        const Type_fr_upper_neck_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_upper_tail_link : public TransformMotion<Scalar, Type_fr_chassis_link_X_fr_upper_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_upper_tail_link();
        const Type_fr_chassis_link_X_fr_upper_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_chassis_link : public TransformMotion<Scalar, Type_fr_upper_tail_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_chassis_link();
        const Type_fr_upper_tail_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_chassis_link_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_chassis_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_chassis_link_COM();
        const Type_fr_world_X_fr_chassis_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_chassis_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_COM_X_fr_world();
        const Type_fr_chassis_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_fixed_base_link : public TransformMotion<Scalar, Type_fr_world_X_fr_fixed_base_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_fixed_base_link();
        const Type_fr_world_X_fr_fixed_base_link& update(const JState&);
    protected:
    };
    
    class Type_fr_fixed_base_link_X_fr_world : public TransformMotion<Scalar, Type_fr_fixed_base_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_fixed_base_link_X_fr_world();
        const Type_fr_fixed_base_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_head_camera_link : public TransformMotion<Scalar, Type_fr_head_link_X_fr_head_camera_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_head_camera_link();
        const Type_fr_head_link_X_fr_head_camera_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_head_link : public TransformMotion<Scalar, Type_fr_head_camera_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_head_link();
        const Type_fr_head_camera_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_head_link_COM : public TransformMotion<Scalar, Type_fr_head_link_X_fr_head_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_head_link_COM();
        const Type_fr_head_link_X_fr_head_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_head_link : public TransformMotion<Scalar, Type_fr_head_link_COM_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_head_link();
        const Type_fr_head_link_COM_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_imu_link : public TransformMotion<Scalar, Type_fr_chassis_link_X_fr_imu_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_imu_link();
        const Type_fr_chassis_link_X_fr_imu_link& update(const JState&);
    protected:
    };
    
    class Type_fr_imu_link_X_fr_chassis_link : public TransformMotion<Scalar, Type_fr_imu_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_imu_link_X_fr_chassis_link();
        const Type_fr_imu_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_left_ear_link_COM : public TransformMotion<Scalar, Type_fr_head_link_X_fr_left_ear_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_left_ear_link_COM();
        const Type_fr_head_link_X_fr_left_ear_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_head_link : public TransformMotion<Scalar, Type_fr_left_ear_link_COM_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_head_link();
        const Type_fr_left_ear_link_COM_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_left_wheel_link_COM : public TransformMotion<Scalar, Type_fr_chassis_link_X_fr_left_wheel_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_left_wheel_link_COM();
        const Type_fr_chassis_link_X_fr_left_wheel_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_COM_X_fr_chassis_link : public TransformMotion<Scalar, Type_fr_left_wheel_link_COM_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_COM_X_fr_chassis_link();
        const Type_fr_left_wheel_link_COM_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_lower_neck_link_COM : public TransformMotion<Scalar, Type_fr_chassis_link_X_fr_lower_neck_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_lower_neck_link_COM();
        const Type_fr_chassis_link_X_fr_lower_neck_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_COM_X_fr_chassis_link : public TransformMotion<Scalar, Type_fr_lower_neck_link_COM_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_COM_X_fr_chassis_link();
        const Type_fr_lower_neck_link_COM_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_lower_tail_link_COM : public TransformMotion<Scalar, Type_fr_upper_tail_link_X_fr_lower_tail_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_lower_tail_link_COM();
        const Type_fr_upper_tail_link_X_fr_lower_tail_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_upper_tail_link : public TransformMotion<Scalar, Type_fr_lower_tail_link_COM_X_fr_upper_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_upper_tail_link();
        const Type_fr_lower_tail_link_COM_X_fr_upper_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_mouth_link_COM : public TransformMotion<Scalar, Type_fr_head_link_X_fr_mouth_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_mouth_link_COM();
        const Type_fr_head_link_X_fr_mouth_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_head_link : public TransformMotion<Scalar, Type_fr_mouth_link_COM_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_head_link();
        const Type_fr_mouth_link_COM_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_parent_link : public TransformMotion<Scalar, Type_fr_world_X_fr_parent_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_parent_link();
        const Type_fr_world_X_fr_parent_link& update(const JState&);
    protected:
    };
    
    class Type_fr_parent_link_X_fr_world : public TransformMotion<Scalar, Type_fr_parent_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_parent_link_X_fr_world();
        const Type_fr_parent_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_right_ear_link_COM : public TransformMotion<Scalar, Type_fr_head_link_X_fr_right_ear_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_right_ear_link_COM();
        const Type_fr_head_link_X_fr_right_ear_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_head_link : public TransformMotion<Scalar, Type_fr_right_ear_link_COM_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_head_link();
        const Type_fr_right_ear_link_COM_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_right_wheel_link_COM : public TransformMotion<Scalar, Type_fr_chassis_link_X_fr_right_wheel_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_right_wheel_link_COM();
        const Type_fr_chassis_link_X_fr_right_wheel_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_COM_X_fr_chassis_link : public TransformMotion<Scalar, Type_fr_right_wheel_link_COM_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_COM_X_fr_chassis_link();
        const Type_fr_right_wheel_link_COM_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_upper_neck_link_COM : public TransformMotion<Scalar, Type_fr_upper_neck_link_X_fr_upper_neck_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_upper_neck_link_COM();
        const Type_fr_upper_neck_link_X_fr_upper_neck_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_upper_neck_link : public TransformMotion<Scalar, Type_fr_upper_neck_link_COM_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_upper_neck_link();
        const Type_fr_upper_neck_link_COM_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_upper_tail_link_COM : public TransformMotion<Scalar, Type_fr_chassis_link_X_fr_upper_tail_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_upper_tail_link_COM();
        const Type_fr_chassis_link_X_fr_upper_tail_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_COM_X_fr_chassis_link : public TransformMotion<Scalar, Type_fr_upper_tail_link_COM_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_COM_X_fr_chassis_link();
        const Type_fr_upper_tail_link_COM_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_world_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_world_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_world_COM();
        const Type_fr_world_X_fr_world_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_world_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_world_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_COM_X_fr_world();
        const Type_fr_world_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_world_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_chassis_joint();
        const Type_fr_world_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_chassis_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_chassis_joint();
        const Type_fr_chassis_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_head_link : public TransformMotion<Scalar, Type_fr_world_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_head_link();
        const Type_fr_world_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_world_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_neck_joint();
        const Type_fr_world_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_neck_joint : public TransformMotion<Scalar, Type_fr_world_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_neck_joint();
        const Type_fr_world_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_head_joint : public TransformMotion<Scalar, Type_fr_world_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_head_joint();
        const Type_fr_world_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_world : public TransformMotion<Scalar, Type_fr_head_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_world();
        const Type_fr_head_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_head_joint : public TransformMotion<Scalar, Type_fr_head_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_head_joint();
        const Type_fr_head_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_upper_neck_joint : public TransformMotion<Scalar, Type_fr_head_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_upper_neck_joint();
        const Type_fr_head_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_head_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_lower_neck_joint();
        const Type_fr_head_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_head_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_chassis_joint();
        const Type_fr_head_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_ear_link : public TransformMotion<Scalar, Type_fr_world_X_fr_left_ear_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_ear_link();
        const Type_fr_world_X_fr_left_ear_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_ear_joint : public TransformMotion<Scalar, Type_fr_world_X_fr_left_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_ear_joint();
        const Type_fr_world_X_fr_left_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_world : public TransformMotion<Scalar, Type_fr_left_ear_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_world();
        const Type_fr_left_ear_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_left_ear_joint : public TransformMotion<Scalar, Type_fr_left_ear_link_X_fr_left_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_left_ear_joint();
        const Type_fr_left_ear_link_X_fr_left_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_head_joint : public TransformMotion<Scalar, Type_fr_left_ear_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_head_joint();
        const Type_fr_left_ear_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_upper_neck_joint : public TransformMotion<Scalar, Type_fr_left_ear_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_upper_neck_joint();
        const Type_fr_left_ear_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_left_ear_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_lower_neck_joint();
        const Type_fr_left_ear_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_left_ear_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_chassis_joint();
        const Type_fr_left_ear_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_wheel_link : public TransformMotion<Scalar, Type_fr_world_X_fr_left_wheel_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_wheel_link();
        const Type_fr_world_X_fr_left_wheel_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_wheel_joint : public TransformMotion<Scalar, Type_fr_world_X_fr_left_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_wheel_joint();
        const Type_fr_world_X_fr_left_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_X_fr_world : public TransformMotion<Scalar, Type_fr_left_wheel_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_X_fr_world();
        const Type_fr_left_wheel_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_X_fr_left_wheel_joint : public TransformMotion<Scalar, Type_fr_left_wheel_link_X_fr_left_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_X_fr_left_wheel_joint();
        const Type_fr_left_wheel_link_X_fr_left_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_left_wheel_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_X_fr_chassis_joint();
        const Type_fr_left_wheel_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_neck_link : public TransformMotion<Scalar, Type_fr_world_X_fr_lower_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_neck_link();
        const Type_fr_world_X_fr_lower_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_world : public TransformMotion<Scalar, Type_fr_lower_neck_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_world();
        const Type_fr_lower_neck_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_lower_neck_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_lower_neck_joint();
        const Type_fr_lower_neck_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_lower_neck_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_chassis_joint();
        const Type_fr_lower_neck_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_tail_link : public TransformMotion<Scalar, Type_fr_world_X_fr_lower_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_tail_link();
        const Type_fr_world_X_fr_lower_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_tail_joint : public TransformMotion<Scalar, Type_fr_world_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_tail_joint();
        const Type_fr_world_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_tail_joint : public TransformMotion<Scalar, Type_fr_world_X_fr_lower_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_tail_joint();
        const Type_fr_world_X_fr_lower_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_world : public TransformMotion<Scalar, Type_fr_lower_tail_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_world();
        const Type_fr_lower_tail_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_lower_tail_joint : public TransformMotion<Scalar, Type_fr_lower_tail_link_X_fr_lower_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_lower_tail_joint();
        const Type_fr_lower_tail_link_X_fr_lower_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_upper_tail_joint : public TransformMotion<Scalar, Type_fr_lower_tail_link_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_upper_tail_joint();
        const Type_fr_lower_tail_link_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_lower_tail_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_chassis_joint();
        const Type_fr_lower_tail_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_mouth_link : public TransformMotion<Scalar, Type_fr_world_X_fr_mouth_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_mouth_link();
        const Type_fr_world_X_fr_mouth_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_mouth_joint : public TransformMotion<Scalar, Type_fr_world_X_fr_mouth_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_mouth_joint();
        const Type_fr_world_X_fr_mouth_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_world : public TransformMotion<Scalar, Type_fr_mouth_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_world();
        const Type_fr_mouth_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_mouth_joint : public TransformMotion<Scalar, Type_fr_mouth_link_X_fr_mouth_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_mouth_joint();
        const Type_fr_mouth_link_X_fr_mouth_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_head_joint : public TransformMotion<Scalar, Type_fr_mouth_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_head_joint();
        const Type_fr_mouth_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_upper_neck_joint : public TransformMotion<Scalar, Type_fr_mouth_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_upper_neck_joint();
        const Type_fr_mouth_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_mouth_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_lower_neck_joint();
        const Type_fr_mouth_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_mouth_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_chassis_joint();
        const Type_fr_mouth_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_ear_link : public TransformMotion<Scalar, Type_fr_world_X_fr_right_ear_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_ear_link();
        const Type_fr_world_X_fr_right_ear_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_ear_joint : public TransformMotion<Scalar, Type_fr_world_X_fr_right_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_ear_joint();
        const Type_fr_world_X_fr_right_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_world : public TransformMotion<Scalar, Type_fr_right_ear_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_world();
        const Type_fr_right_ear_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_right_ear_joint : public TransformMotion<Scalar, Type_fr_right_ear_link_X_fr_right_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_right_ear_joint();
        const Type_fr_right_ear_link_X_fr_right_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_head_joint : public TransformMotion<Scalar, Type_fr_right_ear_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_head_joint();
        const Type_fr_right_ear_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_upper_neck_joint : public TransformMotion<Scalar, Type_fr_right_ear_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_upper_neck_joint();
        const Type_fr_right_ear_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_right_ear_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_lower_neck_joint();
        const Type_fr_right_ear_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_right_ear_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_chassis_joint();
        const Type_fr_right_ear_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_wheel_link : public TransformMotion<Scalar, Type_fr_world_X_fr_right_wheel_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_wheel_link();
        const Type_fr_world_X_fr_right_wheel_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_wheel_joint : public TransformMotion<Scalar, Type_fr_world_X_fr_right_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_wheel_joint();
        const Type_fr_world_X_fr_right_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_X_fr_world : public TransformMotion<Scalar, Type_fr_right_wheel_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_X_fr_world();
        const Type_fr_right_wheel_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_X_fr_right_wheel_joint : public TransformMotion<Scalar, Type_fr_right_wheel_link_X_fr_right_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_X_fr_right_wheel_joint();
        const Type_fr_right_wheel_link_X_fr_right_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_right_wheel_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_X_fr_chassis_joint();
        const Type_fr_right_wheel_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_neck_link : public TransformMotion<Scalar, Type_fr_world_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_neck_link();
        const Type_fr_world_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_world : public TransformMotion<Scalar, Type_fr_upper_neck_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_world();
        const Type_fr_upper_neck_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_upper_neck_joint : public TransformMotion<Scalar, Type_fr_upper_neck_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_upper_neck_joint();
        const Type_fr_upper_neck_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_upper_neck_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_lower_neck_joint();
        const Type_fr_upper_neck_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_upper_neck_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_chassis_joint();
        const Type_fr_upper_neck_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_tail_link : public TransformMotion<Scalar, Type_fr_world_X_fr_upper_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_tail_link();
        const Type_fr_world_X_fr_upper_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_world : public TransformMotion<Scalar, Type_fr_upper_tail_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_world();
        const Type_fr_upper_tail_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_upper_tail_joint : public TransformMotion<Scalar, Type_fr_upper_tail_link_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_upper_tail_joint();
        const Type_fr_upper_tail_link_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_upper_tail_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_chassis_joint();
        const Type_fr_upper_tail_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_COM_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_chassis_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_COM_X_fr_chassis_joint();
        const Type_fr_chassis_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_head_camera_link : public TransformMotion<Scalar, Type_fr_world_X_fr_head_camera_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_head_camera_link();
        const Type_fr_world_X_fr_head_camera_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_world : public TransformMotion<Scalar, Type_fr_head_camera_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_world();
        const Type_fr_head_camera_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_head_joint : public TransformMotion<Scalar, Type_fr_head_camera_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_head_joint();
        const Type_fr_head_camera_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_upper_neck_joint : public TransformMotion<Scalar, Type_fr_head_camera_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_upper_neck_joint();
        const Type_fr_head_camera_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_head_camera_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_lower_neck_joint();
        const Type_fr_head_camera_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_head_camera_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_chassis_joint();
        const Type_fr_head_camera_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_head_link_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_head_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_head_link_COM();
        const Type_fr_world_X_fr_head_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_head_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_world();
        const Type_fr_head_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_head_joint : public TransformMotion<Scalar, Type_fr_head_link_COM_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_head_joint();
        const Type_fr_head_link_COM_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_upper_neck_joint : public TransformMotion<Scalar, Type_fr_head_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_upper_neck_joint();
        const Type_fr_head_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_head_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_lower_neck_joint();
        const Type_fr_head_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_head_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_chassis_joint();
        const Type_fr_head_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_imu_link : public TransformMotion<Scalar, Type_fr_world_X_fr_imu_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_imu_link();
        const Type_fr_world_X_fr_imu_link& update(const JState&);
    protected:
    };
    
    class Type_fr_imu_link_X_fr_world : public TransformMotion<Scalar, Type_fr_imu_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_imu_link_X_fr_world();
        const Type_fr_imu_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_imu_link_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_imu_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_imu_link_X_fr_chassis_joint();
        const Type_fr_imu_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_ear_link_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_left_ear_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_ear_link_COM();
        const Type_fr_world_X_fr_left_ear_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_left_ear_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_world();
        const Type_fr_left_ear_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_left_ear_joint : public TransformMotion<Scalar, Type_fr_left_ear_link_COM_X_fr_left_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_left_ear_joint();
        const Type_fr_left_ear_link_COM_X_fr_left_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_head_joint : public TransformMotion<Scalar, Type_fr_left_ear_link_COM_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_head_joint();
        const Type_fr_left_ear_link_COM_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_upper_neck_joint : public TransformMotion<Scalar, Type_fr_left_ear_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_upper_neck_joint();
        const Type_fr_left_ear_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_left_ear_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_lower_neck_joint();
        const Type_fr_left_ear_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_left_ear_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_chassis_joint();
        const Type_fr_left_ear_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_wheel_link_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_left_wheel_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_wheel_link_COM();
        const Type_fr_world_X_fr_left_wheel_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_left_wheel_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_COM_X_fr_world();
        const Type_fr_left_wheel_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint : public TransformMotion<Scalar, Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint();
        const Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_COM_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_left_wheel_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_COM_X_fr_chassis_joint();
        const Type_fr_left_wheel_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_neck_link_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_lower_neck_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_neck_link_COM();
        const Type_fr_world_X_fr_lower_neck_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_lower_neck_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_COM_X_fr_world();
        const Type_fr_lower_neck_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint();
        const Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_COM_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_lower_neck_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_COM_X_fr_chassis_joint();
        const Type_fr_lower_neck_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_tail_link_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_lower_tail_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_tail_link_COM();
        const Type_fr_world_X_fr_lower_tail_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_lower_tail_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_world();
        const Type_fr_lower_tail_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint : public TransformMotion<Scalar, Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint();
        const Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint : public TransformMotion<Scalar, Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint();
        const Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_lower_tail_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_chassis_joint();
        const Type_fr_lower_tail_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_mouth_link_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_mouth_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_mouth_link_COM();
        const Type_fr_world_X_fr_mouth_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_mouth_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_world();
        const Type_fr_mouth_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_mouth_joint : public TransformMotion<Scalar, Type_fr_mouth_link_COM_X_fr_mouth_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_mouth_joint();
        const Type_fr_mouth_link_COM_X_fr_mouth_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_head_joint : public TransformMotion<Scalar, Type_fr_mouth_link_COM_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_head_joint();
        const Type_fr_mouth_link_COM_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_upper_neck_joint : public TransformMotion<Scalar, Type_fr_mouth_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_upper_neck_joint();
        const Type_fr_mouth_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_mouth_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_lower_neck_joint();
        const Type_fr_mouth_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_mouth_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_chassis_joint();
        const Type_fr_mouth_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_ear_link_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_right_ear_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_ear_link_COM();
        const Type_fr_world_X_fr_right_ear_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_right_ear_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_world();
        const Type_fr_right_ear_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_right_ear_joint : public TransformMotion<Scalar, Type_fr_right_ear_link_COM_X_fr_right_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_right_ear_joint();
        const Type_fr_right_ear_link_COM_X_fr_right_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_head_joint : public TransformMotion<Scalar, Type_fr_right_ear_link_COM_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_head_joint();
        const Type_fr_right_ear_link_COM_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_upper_neck_joint : public TransformMotion<Scalar, Type_fr_right_ear_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_upper_neck_joint();
        const Type_fr_right_ear_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_right_ear_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_lower_neck_joint();
        const Type_fr_right_ear_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_right_ear_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_chassis_joint();
        const Type_fr_right_ear_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_wheel_link_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_right_wheel_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_wheel_link_COM();
        const Type_fr_world_X_fr_right_wheel_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_right_wheel_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_COM_X_fr_world();
        const Type_fr_right_wheel_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint : public TransformMotion<Scalar, Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint();
        const Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_COM_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_right_wheel_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_COM_X_fr_chassis_joint();
        const Type_fr_right_wheel_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_neck_link_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_upper_neck_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_neck_link_COM();
        const Type_fr_world_X_fr_upper_neck_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_upper_neck_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_world();
        const Type_fr_upper_neck_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint : public TransformMotion<Scalar, Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint();
        const Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint : public TransformMotion<Scalar, Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint();
        const Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_upper_neck_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_chassis_joint();
        const Type_fr_upper_neck_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_tail_link_COM : public TransformMotion<Scalar, Type_fr_world_X_fr_upper_tail_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_tail_link_COM();
        const Type_fr_world_X_fr_upper_tail_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_COM_X_fr_world : public TransformMotion<Scalar, Type_fr_upper_tail_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_COM_X_fr_world();
        const Type_fr_upper_tail_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint : public TransformMotion<Scalar, Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint();
        const Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_COM_X_fr_chassis_joint : public TransformMotion<Scalar, Type_fr_upper_tail_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_COM_X_fr_chassis_joint();
        const Type_fr_upper_tail_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_lower_neck_link : public TransformMotion<Scalar, Type_fr_upper_neck_link_X_fr_lower_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_lower_neck_link();
        const Type_fr_upper_neck_link_X_fr_lower_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_upper_neck_link : public TransformMotion<Scalar, Type_fr_lower_neck_link_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_upper_neck_link();
        const Type_fr_lower_neck_link_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
public:
    MotionTransforms();
    void updateParameters();
    Type_fr_world_X_fr_chassis_link fr_world_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_world fr_chassis_link_X_fr_world;
    Type_fr_upper_neck_link_X_fr_head_link fr_upper_neck_link_X_fr_head_link;
    Type_fr_head_link_X_fr_upper_neck_link fr_head_link_X_fr_upper_neck_link;
    Type_fr_head_link_X_fr_left_ear_link fr_head_link_X_fr_left_ear_link;
    Type_fr_left_ear_link_X_fr_head_link fr_left_ear_link_X_fr_head_link;
    Type_fr_chassis_link_X_fr_left_wheel_link fr_chassis_link_X_fr_left_wheel_link;
    Type_fr_left_wheel_link_X_fr_chassis_link fr_left_wheel_link_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_lower_neck_link fr_chassis_link_X_fr_lower_neck_link;
    Type_fr_lower_neck_link_X_fr_chassis_link fr_lower_neck_link_X_fr_chassis_link;
    Type_fr_upper_tail_link_X_fr_lower_tail_link fr_upper_tail_link_X_fr_lower_tail_link;
    Type_fr_lower_tail_link_X_fr_upper_tail_link fr_lower_tail_link_X_fr_upper_tail_link;
    Type_fr_head_link_X_fr_mouth_link fr_head_link_X_fr_mouth_link;
    Type_fr_mouth_link_X_fr_head_link fr_mouth_link_X_fr_head_link;
    Type_fr_head_link_X_fr_right_ear_link fr_head_link_X_fr_right_ear_link;
    Type_fr_right_ear_link_X_fr_head_link fr_right_ear_link_X_fr_head_link;
    Type_fr_chassis_link_X_fr_right_wheel_link fr_chassis_link_X_fr_right_wheel_link;
    Type_fr_right_wheel_link_X_fr_chassis_link fr_right_wheel_link_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_upper_neck_link fr_chassis_link_X_fr_upper_neck_link;
    Type_fr_upper_neck_link_X_fr_chassis_link fr_upper_neck_link_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_upper_tail_link fr_chassis_link_X_fr_upper_tail_link;
    Type_fr_upper_tail_link_X_fr_chassis_link fr_upper_tail_link_X_fr_chassis_link;
    Type_fr_world_X_fr_chassis_link_COM fr_world_X_fr_chassis_link_COM;
    Type_fr_chassis_link_COM_X_fr_world fr_chassis_link_COM_X_fr_world;
    Type_fr_world_X_fr_fixed_base_link fr_world_X_fr_fixed_base_link;
    Type_fr_fixed_base_link_X_fr_world fr_fixed_base_link_X_fr_world;
    Type_fr_head_link_X_fr_head_camera_link fr_head_link_X_fr_head_camera_link;
    Type_fr_head_camera_link_X_fr_head_link fr_head_camera_link_X_fr_head_link;
    Type_fr_head_link_X_fr_head_link_COM fr_head_link_X_fr_head_link_COM;
    Type_fr_head_link_COM_X_fr_head_link fr_head_link_COM_X_fr_head_link;
    Type_fr_chassis_link_X_fr_imu_link fr_chassis_link_X_fr_imu_link;
    Type_fr_imu_link_X_fr_chassis_link fr_imu_link_X_fr_chassis_link;
    Type_fr_head_link_X_fr_left_ear_link_COM fr_head_link_X_fr_left_ear_link_COM;
    Type_fr_left_ear_link_COM_X_fr_head_link fr_left_ear_link_COM_X_fr_head_link;
    Type_fr_chassis_link_X_fr_left_wheel_link_COM fr_chassis_link_X_fr_left_wheel_link_COM;
    Type_fr_left_wheel_link_COM_X_fr_chassis_link fr_left_wheel_link_COM_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_lower_neck_link_COM fr_chassis_link_X_fr_lower_neck_link_COM;
    Type_fr_lower_neck_link_COM_X_fr_chassis_link fr_lower_neck_link_COM_X_fr_chassis_link;
    Type_fr_upper_tail_link_X_fr_lower_tail_link_COM fr_upper_tail_link_X_fr_lower_tail_link_COM;
    Type_fr_lower_tail_link_COM_X_fr_upper_tail_link fr_lower_tail_link_COM_X_fr_upper_tail_link;
    Type_fr_head_link_X_fr_mouth_link_COM fr_head_link_X_fr_mouth_link_COM;
    Type_fr_mouth_link_COM_X_fr_head_link fr_mouth_link_COM_X_fr_head_link;
    Type_fr_world_X_fr_parent_link fr_world_X_fr_parent_link;
    Type_fr_parent_link_X_fr_world fr_parent_link_X_fr_world;
    Type_fr_head_link_X_fr_right_ear_link_COM fr_head_link_X_fr_right_ear_link_COM;
    Type_fr_right_ear_link_COM_X_fr_head_link fr_right_ear_link_COM_X_fr_head_link;
    Type_fr_chassis_link_X_fr_right_wheel_link_COM fr_chassis_link_X_fr_right_wheel_link_COM;
    Type_fr_right_wheel_link_COM_X_fr_chassis_link fr_right_wheel_link_COM_X_fr_chassis_link;
    Type_fr_upper_neck_link_X_fr_upper_neck_link_COM fr_upper_neck_link_X_fr_upper_neck_link_COM;
    Type_fr_upper_neck_link_COM_X_fr_upper_neck_link fr_upper_neck_link_COM_X_fr_upper_neck_link;
    Type_fr_chassis_link_X_fr_upper_tail_link_COM fr_chassis_link_X_fr_upper_tail_link_COM;
    Type_fr_upper_tail_link_COM_X_fr_chassis_link fr_upper_tail_link_COM_X_fr_chassis_link;
    Type_fr_world_X_fr_world_COM fr_world_X_fr_world_COM;
    Type_fr_world_COM_X_fr_world fr_world_COM_X_fr_world;
    Type_fr_world_X_fr_chassis_joint fr_world_X_fr_chassis_joint;
    Type_fr_chassis_link_X_fr_chassis_joint fr_chassis_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_head_link fr_world_X_fr_head_link;
    Type_fr_world_X_fr_lower_neck_joint fr_world_X_fr_lower_neck_joint;
    Type_fr_world_X_fr_upper_neck_joint fr_world_X_fr_upper_neck_joint;
    Type_fr_world_X_fr_head_joint fr_world_X_fr_head_joint;
    Type_fr_head_link_X_fr_world fr_head_link_X_fr_world;
    Type_fr_head_link_X_fr_head_joint fr_head_link_X_fr_head_joint;
    Type_fr_head_link_X_fr_upper_neck_joint fr_head_link_X_fr_upper_neck_joint;
    Type_fr_head_link_X_fr_lower_neck_joint fr_head_link_X_fr_lower_neck_joint;
    Type_fr_head_link_X_fr_chassis_joint fr_head_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_left_ear_link fr_world_X_fr_left_ear_link;
    Type_fr_world_X_fr_left_ear_joint fr_world_X_fr_left_ear_joint;
    Type_fr_left_ear_link_X_fr_world fr_left_ear_link_X_fr_world;
    Type_fr_left_ear_link_X_fr_left_ear_joint fr_left_ear_link_X_fr_left_ear_joint;
    Type_fr_left_ear_link_X_fr_head_joint fr_left_ear_link_X_fr_head_joint;
    Type_fr_left_ear_link_X_fr_upper_neck_joint fr_left_ear_link_X_fr_upper_neck_joint;
    Type_fr_left_ear_link_X_fr_lower_neck_joint fr_left_ear_link_X_fr_lower_neck_joint;
    Type_fr_left_ear_link_X_fr_chassis_joint fr_left_ear_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_left_wheel_link fr_world_X_fr_left_wheel_link;
    Type_fr_world_X_fr_left_wheel_joint fr_world_X_fr_left_wheel_joint;
    Type_fr_left_wheel_link_X_fr_world fr_left_wheel_link_X_fr_world;
    Type_fr_left_wheel_link_X_fr_left_wheel_joint fr_left_wheel_link_X_fr_left_wheel_joint;
    Type_fr_left_wheel_link_X_fr_chassis_joint fr_left_wheel_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_lower_neck_link fr_world_X_fr_lower_neck_link;
    Type_fr_lower_neck_link_X_fr_world fr_lower_neck_link_X_fr_world;
    Type_fr_lower_neck_link_X_fr_lower_neck_joint fr_lower_neck_link_X_fr_lower_neck_joint;
    Type_fr_lower_neck_link_X_fr_chassis_joint fr_lower_neck_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_lower_tail_link fr_world_X_fr_lower_tail_link;
    Type_fr_world_X_fr_upper_tail_joint fr_world_X_fr_upper_tail_joint;
    Type_fr_world_X_fr_lower_tail_joint fr_world_X_fr_lower_tail_joint;
    Type_fr_lower_tail_link_X_fr_world fr_lower_tail_link_X_fr_world;
    Type_fr_lower_tail_link_X_fr_lower_tail_joint fr_lower_tail_link_X_fr_lower_tail_joint;
    Type_fr_lower_tail_link_X_fr_upper_tail_joint fr_lower_tail_link_X_fr_upper_tail_joint;
    Type_fr_lower_tail_link_X_fr_chassis_joint fr_lower_tail_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_mouth_link fr_world_X_fr_mouth_link;
    Type_fr_world_X_fr_mouth_joint fr_world_X_fr_mouth_joint;
    Type_fr_mouth_link_X_fr_world fr_mouth_link_X_fr_world;
    Type_fr_mouth_link_X_fr_mouth_joint fr_mouth_link_X_fr_mouth_joint;
    Type_fr_mouth_link_X_fr_head_joint fr_mouth_link_X_fr_head_joint;
    Type_fr_mouth_link_X_fr_upper_neck_joint fr_mouth_link_X_fr_upper_neck_joint;
    Type_fr_mouth_link_X_fr_lower_neck_joint fr_mouth_link_X_fr_lower_neck_joint;
    Type_fr_mouth_link_X_fr_chassis_joint fr_mouth_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_right_ear_link fr_world_X_fr_right_ear_link;
    Type_fr_world_X_fr_right_ear_joint fr_world_X_fr_right_ear_joint;
    Type_fr_right_ear_link_X_fr_world fr_right_ear_link_X_fr_world;
    Type_fr_right_ear_link_X_fr_right_ear_joint fr_right_ear_link_X_fr_right_ear_joint;
    Type_fr_right_ear_link_X_fr_head_joint fr_right_ear_link_X_fr_head_joint;
    Type_fr_right_ear_link_X_fr_upper_neck_joint fr_right_ear_link_X_fr_upper_neck_joint;
    Type_fr_right_ear_link_X_fr_lower_neck_joint fr_right_ear_link_X_fr_lower_neck_joint;
    Type_fr_right_ear_link_X_fr_chassis_joint fr_right_ear_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_right_wheel_link fr_world_X_fr_right_wheel_link;
    Type_fr_world_X_fr_right_wheel_joint fr_world_X_fr_right_wheel_joint;
    Type_fr_right_wheel_link_X_fr_world fr_right_wheel_link_X_fr_world;
    Type_fr_right_wheel_link_X_fr_right_wheel_joint fr_right_wheel_link_X_fr_right_wheel_joint;
    Type_fr_right_wheel_link_X_fr_chassis_joint fr_right_wheel_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_upper_neck_link fr_world_X_fr_upper_neck_link;
    Type_fr_upper_neck_link_X_fr_world fr_upper_neck_link_X_fr_world;
    Type_fr_upper_neck_link_X_fr_upper_neck_joint fr_upper_neck_link_X_fr_upper_neck_joint;
    Type_fr_upper_neck_link_X_fr_lower_neck_joint fr_upper_neck_link_X_fr_lower_neck_joint;
    Type_fr_upper_neck_link_X_fr_chassis_joint fr_upper_neck_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_upper_tail_link fr_world_X_fr_upper_tail_link;
    Type_fr_upper_tail_link_X_fr_world fr_upper_tail_link_X_fr_world;
    Type_fr_upper_tail_link_X_fr_upper_tail_joint fr_upper_tail_link_X_fr_upper_tail_joint;
    Type_fr_upper_tail_link_X_fr_chassis_joint fr_upper_tail_link_X_fr_chassis_joint;
    Type_fr_chassis_link_COM_X_fr_chassis_joint fr_chassis_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_head_camera_link fr_world_X_fr_head_camera_link;
    Type_fr_head_camera_link_X_fr_world fr_head_camera_link_X_fr_world;
    Type_fr_head_camera_link_X_fr_head_joint fr_head_camera_link_X_fr_head_joint;
    Type_fr_head_camera_link_X_fr_upper_neck_joint fr_head_camera_link_X_fr_upper_neck_joint;
    Type_fr_head_camera_link_X_fr_lower_neck_joint fr_head_camera_link_X_fr_lower_neck_joint;
    Type_fr_head_camera_link_X_fr_chassis_joint fr_head_camera_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_head_link_COM fr_world_X_fr_head_link_COM;
    Type_fr_head_link_COM_X_fr_world fr_head_link_COM_X_fr_world;
    Type_fr_head_link_COM_X_fr_head_joint fr_head_link_COM_X_fr_head_joint;
    Type_fr_head_link_COM_X_fr_upper_neck_joint fr_head_link_COM_X_fr_upper_neck_joint;
    Type_fr_head_link_COM_X_fr_lower_neck_joint fr_head_link_COM_X_fr_lower_neck_joint;
    Type_fr_head_link_COM_X_fr_chassis_joint fr_head_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_imu_link fr_world_X_fr_imu_link;
    Type_fr_imu_link_X_fr_world fr_imu_link_X_fr_world;
    Type_fr_imu_link_X_fr_chassis_joint fr_imu_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_left_ear_link_COM fr_world_X_fr_left_ear_link_COM;
    Type_fr_left_ear_link_COM_X_fr_world fr_left_ear_link_COM_X_fr_world;
    Type_fr_left_ear_link_COM_X_fr_left_ear_joint fr_left_ear_link_COM_X_fr_left_ear_joint;
    Type_fr_left_ear_link_COM_X_fr_head_joint fr_left_ear_link_COM_X_fr_head_joint;
    Type_fr_left_ear_link_COM_X_fr_upper_neck_joint fr_left_ear_link_COM_X_fr_upper_neck_joint;
    Type_fr_left_ear_link_COM_X_fr_lower_neck_joint fr_left_ear_link_COM_X_fr_lower_neck_joint;
    Type_fr_left_ear_link_COM_X_fr_chassis_joint fr_left_ear_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_left_wheel_link_COM fr_world_X_fr_left_wheel_link_COM;
    Type_fr_left_wheel_link_COM_X_fr_world fr_left_wheel_link_COM_X_fr_world;
    Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint fr_left_wheel_link_COM_X_fr_left_wheel_joint;
    Type_fr_left_wheel_link_COM_X_fr_chassis_joint fr_left_wheel_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_lower_neck_link_COM fr_world_X_fr_lower_neck_link_COM;
    Type_fr_lower_neck_link_COM_X_fr_world fr_lower_neck_link_COM_X_fr_world;
    Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint fr_lower_neck_link_COM_X_fr_lower_neck_joint;
    Type_fr_lower_neck_link_COM_X_fr_chassis_joint fr_lower_neck_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_lower_tail_link_COM fr_world_X_fr_lower_tail_link_COM;
    Type_fr_lower_tail_link_COM_X_fr_world fr_lower_tail_link_COM_X_fr_world;
    Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint fr_lower_tail_link_COM_X_fr_lower_tail_joint;
    Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint fr_lower_tail_link_COM_X_fr_upper_tail_joint;
    Type_fr_lower_tail_link_COM_X_fr_chassis_joint fr_lower_tail_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_mouth_link_COM fr_world_X_fr_mouth_link_COM;
    Type_fr_mouth_link_COM_X_fr_world fr_mouth_link_COM_X_fr_world;
    Type_fr_mouth_link_COM_X_fr_mouth_joint fr_mouth_link_COM_X_fr_mouth_joint;
    Type_fr_mouth_link_COM_X_fr_head_joint fr_mouth_link_COM_X_fr_head_joint;
    Type_fr_mouth_link_COM_X_fr_upper_neck_joint fr_mouth_link_COM_X_fr_upper_neck_joint;
    Type_fr_mouth_link_COM_X_fr_lower_neck_joint fr_mouth_link_COM_X_fr_lower_neck_joint;
    Type_fr_mouth_link_COM_X_fr_chassis_joint fr_mouth_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_right_ear_link_COM fr_world_X_fr_right_ear_link_COM;
    Type_fr_right_ear_link_COM_X_fr_world fr_right_ear_link_COM_X_fr_world;
    Type_fr_right_ear_link_COM_X_fr_right_ear_joint fr_right_ear_link_COM_X_fr_right_ear_joint;
    Type_fr_right_ear_link_COM_X_fr_head_joint fr_right_ear_link_COM_X_fr_head_joint;
    Type_fr_right_ear_link_COM_X_fr_upper_neck_joint fr_right_ear_link_COM_X_fr_upper_neck_joint;
    Type_fr_right_ear_link_COM_X_fr_lower_neck_joint fr_right_ear_link_COM_X_fr_lower_neck_joint;
    Type_fr_right_ear_link_COM_X_fr_chassis_joint fr_right_ear_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_right_wheel_link_COM fr_world_X_fr_right_wheel_link_COM;
    Type_fr_right_wheel_link_COM_X_fr_world fr_right_wheel_link_COM_X_fr_world;
    Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint fr_right_wheel_link_COM_X_fr_right_wheel_joint;
    Type_fr_right_wheel_link_COM_X_fr_chassis_joint fr_right_wheel_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_upper_neck_link_COM fr_world_X_fr_upper_neck_link_COM;
    Type_fr_upper_neck_link_COM_X_fr_world fr_upper_neck_link_COM_X_fr_world;
    Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint fr_upper_neck_link_COM_X_fr_upper_neck_joint;
    Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint fr_upper_neck_link_COM_X_fr_lower_neck_joint;
    Type_fr_upper_neck_link_COM_X_fr_chassis_joint fr_upper_neck_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_upper_tail_link_COM fr_world_X_fr_upper_tail_link_COM;
    Type_fr_upper_tail_link_COM_X_fr_world fr_upper_tail_link_COM_X_fr_world;
    Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint fr_upper_tail_link_COM_X_fr_upper_tail_joint;
    Type_fr_upper_tail_link_COM_X_fr_chassis_joint fr_upper_tail_link_COM_X_fr_chassis_joint;
    Type_fr_upper_neck_link_X_fr_lower_neck_link fr_upper_neck_link_X_fr_lower_neck_link;
    Type_fr_lower_neck_link_X_fr_upper_neck_link fr_lower_neck_link_X_fr_upper_neck_link;

protected:

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
template <typename TRAIT>
class ForceTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformForce<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_world_X_fr_chassis_link : public TransformForce<Scalar, Type_fr_world_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_chassis_link();
        const Type_fr_world_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_world : public TransformForce<Scalar, Type_fr_chassis_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_world();
        const Type_fr_chassis_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_head_link : public TransformForce<Scalar, Type_fr_upper_neck_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_head_link();
        const Type_fr_upper_neck_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_upper_neck_link : public TransformForce<Scalar, Type_fr_head_link_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_upper_neck_link();
        const Type_fr_head_link_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_left_ear_link : public TransformForce<Scalar, Type_fr_head_link_X_fr_left_ear_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_left_ear_link();
        const Type_fr_head_link_X_fr_left_ear_link& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_head_link : public TransformForce<Scalar, Type_fr_left_ear_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_head_link();
        const Type_fr_left_ear_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_left_wheel_link : public TransformForce<Scalar, Type_fr_chassis_link_X_fr_left_wheel_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_left_wheel_link();
        const Type_fr_chassis_link_X_fr_left_wheel_link& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_X_fr_chassis_link : public TransformForce<Scalar, Type_fr_left_wheel_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_X_fr_chassis_link();
        const Type_fr_left_wheel_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_lower_neck_link : public TransformForce<Scalar, Type_fr_chassis_link_X_fr_lower_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_lower_neck_link();
        const Type_fr_chassis_link_X_fr_lower_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_chassis_link : public TransformForce<Scalar, Type_fr_lower_neck_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_chassis_link();
        const Type_fr_lower_neck_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_lower_tail_link : public TransformForce<Scalar, Type_fr_upper_tail_link_X_fr_lower_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_lower_tail_link();
        const Type_fr_upper_tail_link_X_fr_lower_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_upper_tail_link : public TransformForce<Scalar, Type_fr_lower_tail_link_X_fr_upper_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_upper_tail_link();
        const Type_fr_lower_tail_link_X_fr_upper_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_mouth_link : public TransformForce<Scalar, Type_fr_head_link_X_fr_mouth_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_mouth_link();
        const Type_fr_head_link_X_fr_mouth_link& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_head_link : public TransformForce<Scalar, Type_fr_mouth_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_head_link();
        const Type_fr_mouth_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_right_ear_link : public TransformForce<Scalar, Type_fr_head_link_X_fr_right_ear_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_right_ear_link();
        const Type_fr_head_link_X_fr_right_ear_link& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_head_link : public TransformForce<Scalar, Type_fr_right_ear_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_head_link();
        const Type_fr_right_ear_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_right_wheel_link : public TransformForce<Scalar, Type_fr_chassis_link_X_fr_right_wheel_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_right_wheel_link();
        const Type_fr_chassis_link_X_fr_right_wheel_link& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_X_fr_chassis_link : public TransformForce<Scalar, Type_fr_right_wheel_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_X_fr_chassis_link();
        const Type_fr_right_wheel_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_upper_neck_link : public TransformForce<Scalar, Type_fr_chassis_link_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_upper_neck_link();
        const Type_fr_chassis_link_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_chassis_link : public TransformForce<Scalar, Type_fr_upper_neck_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_chassis_link();
        const Type_fr_upper_neck_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_upper_tail_link : public TransformForce<Scalar, Type_fr_chassis_link_X_fr_upper_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_upper_tail_link();
        const Type_fr_chassis_link_X_fr_upper_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_chassis_link : public TransformForce<Scalar, Type_fr_upper_tail_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_chassis_link();
        const Type_fr_upper_tail_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_chassis_link_COM : public TransformForce<Scalar, Type_fr_world_X_fr_chassis_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_chassis_link_COM();
        const Type_fr_world_X_fr_chassis_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_COM_X_fr_world : public TransformForce<Scalar, Type_fr_chassis_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_COM_X_fr_world();
        const Type_fr_chassis_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_fixed_base_link : public TransformForce<Scalar, Type_fr_world_X_fr_fixed_base_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_fixed_base_link();
        const Type_fr_world_X_fr_fixed_base_link& update(const JState&);
    protected:
    };
    
    class Type_fr_fixed_base_link_X_fr_world : public TransformForce<Scalar, Type_fr_fixed_base_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_fixed_base_link_X_fr_world();
        const Type_fr_fixed_base_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_head_camera_link : public TransformForce<Scalar, Type_fr_head_link_X_fr_head_camera_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_head_camera_link();
        const Type_fr_head_link_X_fr_head_camera_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_head_link : public TransformForce<Scalar, Type_fr_head_camera_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_head_link();
        const Type_fr_head_camera_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_head_link_COM : public TransformForce<Scalar, Type_fr_head_link_X_fr_head_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_head_link_COM();
        const Type_fr_head_link_X_fr_head_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_head_link : public TransformForce<Scalar, Type_fr_head_link_COM_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_head_link();
        const Type_fr_head_link_COM_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_imu_link : public TransformForce<Scalar, Type_fr_chassis_link_X_fr_imu_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_imu_link();
        const Type_fr_chassis_link_X_fr_imu_link& update(const JState&);
    protected:
    };
    
    class Type_fr_imu_link_X_fr_chassis_link : public TransformForce<Scalar, Type_fr_imu_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_imu_link_X_fr_chassis_link();
        const Type_fr_imu_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_left_ear_link_COM : public TransformForce<Scalar, Type_fr_head_link_X_fr_left_ear_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_left_ear_link_COM();
        const Type_fr_head_link_X_fr_left_ear_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_head_link : public TransformForce<Scalar, Type_fr_left_ear_link_COM_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_head_link();
        const Type_fr_left_ear_link_COM_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_left_wheel_link_COM : public TransformForce<Scalar, Type_fr_chassis_link_X_fr_left_wheel_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_left_wheel_link_COM();
        const Type_fr_chassis_link_X_fr_left_wheel_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_COM_X_fr_chassis_link : public TransformForce<Scalar, Type_fr_left_wheel_link_COM_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_COM_X_fr_chassis_link();
        const Type_fr_left_wheel_link_COM_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_lower_neck_link_COM : public TransformForce<Scalar, Type_fr_chassis_link_X_fr_lower_neck_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_lower_neck_link_COM();
        const Type_fr_chassis_link_X_fr_lower_neck_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_COM_X_fr_chassis_link : public TransformForce<Scalar, Type_fr_lower_neck_link_COM_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_COM_X_fr_chassis_link();
        const Type_fr_lower_neck_link_COM_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_lower_tail_link_COM : public TransformForce<Scalar, Type_fr_upper_tail_link_X_fr_lower_tail_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_lower_tail_link_COM();
        const Type_fr_upper_tail_link_X_fr_lower_tail_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_upper_tail_link : public TransformForce<Scalar, Type_fr_lower_tail_link_COM_X_fr_upper_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_upper_tail_link();
        const Type_fr_lower_tail_link_COM_X_fr_upper_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_mouth_link_COM : public TransformForce<Scalar, Type_fr_head_link_X_fr_mouth_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_mouth_link_COM();
        const Type_fr_head_link_X_fr_mouth_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_head_link : public TransformForce<Scalar, Type_fr_mouth_link_COM_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_head_link();
        const Type_fr_mouth_link_COM_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_parent_link : public TransformForce<Scalar, Type_fr_world_X_fr_parent_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_parent_link();
        const Type_fr_world_X_fr_parent_link& update(const JState&);
    protected:
    };
    
    class Type_fr_parent_link_X_fr_world : public TransformForce<Scalar, Type_fr_parent_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_parent_link_X_fr_world();
        const Type_fr_parent_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_right_ear_link_COM : public TransformForce<Scalar, Type_fr_head_link_X_fr_right_ear_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_right_ear_link_COM();
        const Type_fr_head_link_X_fr_right_ear_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_head_link : public TransformForce<Scalar, Type_fr_right_ear_link_COM_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_head_link();
        const Type_fr_right_ear_link_COM_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_right_wheel_link_COM : public TransformForce<Scalar, Type_fr_chassis_link_X_fr_right_wheel_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_right_wheel_link_COM();
        const Type_fr_chassis_link_X_fr_right_wheel_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_COM_X_fr_chassis_link : public TransformForce<Scalar, Type_fr_right_wheel_link_COM_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_COM_X_fr_chassis_link();
        const Type_fr_right_wheel_link_COM_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_upper_neck_link_COM : public TransformForce<Scalar, Type_fr_upper_neck_link_X_fr_upper_neck_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_upper_neck_link_COM();
        const Type_fr_upper_neck_link_X_fr_upper_neck_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_upper_neck_link : public TransformForce<Scalar, Type_fr_upper_neck_link_COM_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_upper_neck_link();
        const Type_fr_upper_neck_link_COM_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_upper_tail_link_COM : public TransformForce<Scalar, Type_fr_chassis_link_X_fr_upper_tail_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_upper_tail_link_COM();
        const Type_fr_chassis_link_X_fr_upper_tail_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_COM_X_fr_chassis_link : public TransformForce<Scalar, Type_fr_upper_tail_link_COM_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_COM_X_fr_chassis_link();
        const Type_fr_upper_tail_link_COM_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_world_COM : public TransformForce<Scalar, Type_fr_world_X_fr_world_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_world_COM();
        const Type_fr_world_X_fr_world_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_world_COM_X_fr_world : public TransformForce<Scalar, Type_fr_world_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_COM_X_fr_world();
        const Type_fr_world_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_world_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_chassis_joint();
        const Type_fr_world_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_chassis_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_chassis_joint();
        const Type_fr_chassis_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_head_link : public TransformForce<Scalar, Type_fr_world_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_head_link();
        const Type_fr_world_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_world_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_neck_joint();
        const Type_fr_world_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_neck_joint : public TransformForce<Scalar, Type_fr_world_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_neck_joint();
        const Type_fr_world_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_head_joint : public TransformForce<Scalar, Type_fr_world_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_head_joint();
        const Type_fr_world_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_world : public TransformForce<Scalar, Type_fr_head_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_world();
        const Type_fr_head_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_head_joint : public TransformForce<Scalar, Type_fr_head_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_head_joint();
        const Type_fr_head_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_upper_neck_joint : public TransformForce<Scalar, Type_fr_head_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_upper_neck_joint();
        const Type_fr_head_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_head_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_lower_neck_joint();
        const Type_fr_head_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_head_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_chassis_joint();
        const Type_fr_head_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_ear_link : public TransformForce<Scalar, Type_fr_world_X_fr_left_ear_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_ear_link();
        const Type_fr_world_X_fr_left_ear_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_ear_joint : public TransformForce<Scalar, Type_fr_world_X_fr_left_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_ear_joint();
        const Type_fr_world_X_fr_left_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_world : public TransformForce<Scalar, Type_fr_left_ear_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_world();
        const Type_fr_left_ear_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_left_ear_joint : public TransformForce<Scalar, Type_fr_left_ear_link_X_fr_left_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_left_ear_joint();
        const Type_fr_left_ear_link_X_fr_left_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_head_joint : public TransformForce<Scalar, Type_fr_left_ear_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_head_joint();
        const Type_fr_left_ear_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_upper_neck_joint : public TransformForce<Scalar, Type_fr_left_ear_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_upper_neck_joint();
        const Type_fr_left_ear_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_left_ear_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_lower_neck_joint();
        const Type_fr_left_ear_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_left_ear_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_chassis_joint();
        const Type_fr_left_ear_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_wheel_link : public TransformForce<Scalar, Type_fr_world_X_fr_left_wheel_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_wheel_link();
        const Type_fr_world_X_fr_left_wheel_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_wheel_joint : public TransformForce<Scalar, Type_fr_world_X_fr_left_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_wheel_joint();
        const Type_fr_world_X_fr_left_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_X_fr_world : public TransformForce<Scalar, Type_fr_left_wheel_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_X_fr_world();
        const Type_fr_left_wheel_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_X_fr_left_wheel_joint : public TransformForce<Scalar, Type_fr_left_wheel_link_X_fr_left_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_X_fr_left_wheel_joint();
        const Type_fr_left_wheel_link_X_fr_left_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_left_wheel_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_X_fr_chassis_joint();
        const Type_fr_left_wheel_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_neck_link : public TransformForce<Scalar, Type_fr_world_X_fr_lower_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_neck_link();
        const Type_fr_world_X_fr_lower_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_world : public TransformForce<Scalar, Type_fr_lower_neck_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_world();
        const Type_fr_lower_neck_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_lower_neck_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_lower_neck_joint();
        const Type_fr_lower_neck_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_lower_neck_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_chassis_joint();
        const Type_fr_lower_neck_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_tail_link : public TransformForce<Scalar, Type_fr_world_X_fr_lower_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_tail_link();
        const Type_fr_world_X_fr_lower_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_tail_joint : public TransformForce<Scalar, Type_fr_world_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_tail_joint();
        const Type_fr_world_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_tail_joint : public TransformForce<Scalar, Type_fr_world_X_fr_lower_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_tail_joint();
        const Type_fr_world_X_fr_lower_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_world : public TransformForce<Scalar, Type_fr_lower_tail_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_world();
        const Type_fr_lower_tail_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_lower_tail_joint : public TransformForce<Scalar, Type_fr_lower_tail_link_X_fr_lower_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_lower_tail_joint();
        const Type_fr_lower_tail_link_X_fr_lower_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_upper_tail_joint : public TransformForce<Scalar, Type_fr_lower_tail_link_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_upper_tail_joint();
        const Type_fr_lower_tail_link_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_lower_tail_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_chassis_joint();
        const Type_fr_lower_tail_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_mouth_link : public TransformForce<Scalar, Type_fr_world_X_fr_mouth_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_mouth_link();
        const Type_fr_world_X_fr_mouth_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_mouth_joint : public TransformForce<Scalar, Type_fr_world_X_fr_mouth_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_mouth_joint();
        const Type_fr_world_X_fr_mouth_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_world : public TransformForce<Scalar, Type_fr_mouth_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_world();
        const Type_fr_mouth_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_mouth_joint : public TransformForce<Scalar, Type_fr_mouth_link_X_fr_mouth_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_mouth_joint();
        const Type_fr_mouth_link_X_fr_mouth_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_head_joint : public TransformForce<Scalar, Type_fr_mouth_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_head_joint();
        const Type_fr_mouth_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_upper_neck_joint : public TransformForce<Scalar, Type_fr_mouth_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_upper_neck_joint();
        const Type_fr_mouth_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_mouth_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_lower_neck_joint();
        const Type_fr_mouth_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_mouth_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_chassis_joint();
        const Type_fr_mouth_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_ear_link : public TransformForce<Scalar, Type_fr_world_X_fr_right_ear_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_ear_link();
        const Type_fr_world_X_fr_right_ear_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_ear_joint : public TransformForce<Scalar, Type_fr_world_X_fr_right_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_ear_joint();
        const Type_fr_world_X_fr_right_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_world : public TransformForce<Scalar, Type_fr_right_ear_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_world();
        const Type_fr_right_ear_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_right_ear_joint : public TransformForce<Scalar, Type_fr_right_ear_link_X_fr_right_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_right_ear_joint();
        const Type_fr_right_ear_link_X_fr_right_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_head_joint : public TransformForce<Scalar, Type_fr_right_ear_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_head_joint();
        const Type_fr_right_ear_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_upper_neck_joint : public TransformForce<Scalar, Type_fr_right_ear_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_upper_neck_joint();
        const Type_fr_right_ear_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_right_ear_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_lower_neck_joint();
        const Type_fr_right_ear_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_right_ear_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_chassis_joint();
        const Type_fr_right_ear_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_wheel_link : public TransformForce<Scalar, Type_fr_world_X_fr_right_wheel_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_wheel_link();
        const Type_fr_world_X_fr_right_wheel_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_wheel_joint : public TransformForce<Scalar, Type_fr_world_X_fr_right_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_wheel_joint();
        const Type_fr_world_X_fr_right_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_X_fr_world : public TransformForce<Scalar, Type_fr_right_wheel_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_X_fr_world();
        const Type_fr_right_wheel_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_X_fr_right_wheel_joint : public TransformForce<Scalar, Type_fr_right_wheel_link_X_fr_right_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_X_fr_right_wheel_joint();
        const Type_fr_right_wheel_link_X_fr_right_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_right_wheel_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_X_fr_chassis_joint();
        const Type_fr_right_wheel_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_neck_link : public TransformForce<Scalar, Type_fr_world_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_neck_link();
        const Type_fr_world_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_world : public TransformForce<Scalar, Type_fr_upper_neck_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_world();
        const Type_fr_upper_neck_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_upper_neck_joint : public TransformForce<Scalar, Type_fr_upper_neck_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_upper_neck_joint();
        const Type_fr_upper_neck_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_upper_neck_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_lower_neck_joint();
        const Type_fr_upper_neck_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_upper_neck_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_chassis_joint();
        const Type_fr_upper_neck_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_tail_link : public TransformForce<Scalar, Type_fr_world_X_fr_upper_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_tail_link();
        const Type_fr_world_X_fr_upper_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_world : public TransformForce<Scalar, Type_fr_upper_tail_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_world();
        const Type_fr_upper_tail_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_upper_tail_joint : public TransformForce<Scalar, Type_fr_upper_tail_link_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_upper_tail_joint();
        const Type_fr_upper_tail_link_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_upper_tail_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_chassis_joint();
        const Type_fr_upper_tail_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_COM_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_chassis_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_COM_X_fr_chassis_joint();
        const Type_fr_chassis_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_head_camera_link : public TransformForce<Scalar, Type_fr_world_X_fr_head_camera_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_head_camera_link();
        const Type_fr_world_X_fr_head_camera_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_world : public TransformForce<Scalar, Type_fr_head_camera_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_world();
        const Type_fr_head_camera_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_head_joint : public TransformForce<Scalar, Type_fr_head_camera_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_head_joint();
        const Type_fr_head_camera_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_upper_neck_joint : public TransformForce<Scalar, Type_fr_head_camera_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_upper_neck_joint();
        const Type_fr_head_camera_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_head_camera_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_lower_neck_joint();
        const Type_fr_head_camera_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_head_camera_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_chassis_joint();
        const Type_fr_head_camera_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_head_link_COM : public TransformForce<Scalar, Type_fr_world_X_fr_head_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_head_link_COM();
        const Type_fr_world_X_fr_head_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_world : public TransformForce<Scalar, Type_fr_head_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_world();
        const Type_fr_head_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_head_joint : public TransformForce<Scalar, Type_fr_head_link_COM_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_head_joint();
        const Type_fr_head_link_COM_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_upper_neck_joint : public TransformForce<Scalar, Type_fr_head_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_upper_neck_joint();
        const Type_fr_head_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_head_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_lower_neck_joint();
        const Type_fr_head_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_head_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_chassis_joint();
        const Type_fr_head_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_imu_link : public TransformForce<Scalar, Type_fr_world_X_fr_imu_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_imu_link();
        const Type_fr_world_X_fr_imu_link& update(const JState&);
    protected:
    };
    
    class Type_fr_imu_link_X_fr_world : public TransformForce<Scalar, Type_fr_imu_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_imu_link_X_fr_world();
        const Type_fr_imu_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_imu_link_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_imu_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_imu_link_X_fr_chassis_joint();
        const Type_fr_imu_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_ear_link_COM : public TransformForce<Scalar, Type_fr_world_X_fr_left_ear_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_ear_link_COM();
        const Type_fr_world_X_fr_left_ear_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_world : public TransformForce<Scalar, Type_fr_left_ear_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_world();
        const Type_fr_left_ear_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_left_ear_joint : public TransformForce<Scalar, Type_fr_left_ear_link_COM_X_fr_left_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_left_ear_joint();
        const Type_fr_left_ear_link_COM_X_fr_left_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_head_joint : public TransformForce<Scalar, Type_fr_left_ear_link_COM_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_head_joint();
        const Type_fr_left_ear_link_COM_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_upper_neck_joint : public TransformForce<Scalar, Type_fr_left_ear_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_upper_neck_joint();
        const Type_fr_left_ear_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_left_ear_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_lower_neck_joint();
        const Type_fr_left_ear_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_left_ear_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_chassis_joint();
        const Type_fr_left_ear_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_wheel_link_COM : public TransformForce<Scalar, Type_fr_world_X_fr_left_wheel_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_wheel_link_COM();
        const Type_fr_world_X_fr_left_wheel_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_COM_X_fr_world : public TransformForce<Scalar, Type_fr_left_wheel_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_COM_X_fr_world();
        const Type_fr_left_wheel_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint : public TransformForce<Scalar, Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint();
        const Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_COM_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_left_wheel_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_COM_X_fr_chassis_joint();
        const Type_fr_left_wheel_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_neck_link_COM : public TransformForce<Scalar, Type_fr_world_X_fr_lower_neck_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_neck_link_COM();
        const Type_fr_world_X_fr_lower_neck_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_COM_X_fr_world : public TransformForce<Scalar, Type_fr_lower_neck_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_COM_X_fr_world();
        const Type_fr_lower_neck_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint();
        const Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_COM_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_lower_neck_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_COM_X_fr_chassis_joint();
        const Type_fr_lower_neck_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_tail_link_COM : public TransformForce<Scalar, Type_fr_world_X_fr_lower_tail_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_tail_link_COM();
        const Type_fr_world_X_fr_lower_tail_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_world : public TransformForce<Scalar, Type_fr_lower_tail_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_world();
        const Type_fr_lower_tail_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint : public TransformForce<Scalar, Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint();
        const Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint : public TransformForce<Scalar, Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint();
        const Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_lower_tail_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_chassis_joint();
        const Type_fr_lower_tail_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_mouth_link_COM : public TransformForce<Scalar, Type_fr_world_X_fr_mouth_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_mouth_link_COM();
        const Type_fr_world_X_fr_mouth_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_world : public TransformForce<Scalar, Type_fr_mouth_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_world();
        const Type_fr_mouth_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_mouth_joint : public TransformForce<Scalar, Type_fr_mouth_link_COM_X_fr_mouth_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_mouth_joint();
        const Type_fr_mouth_link_COM_X_fr_mouth_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_head_joint : public TransformForce<Scalar, Type_fr_mouth_link_COM_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_head_joint();
        const Type_fr_mouth_link_COM_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_upper_neck_joint : public TransformForce<Scalar, Type_fr_mouth_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_upper_neck_joint();
        const Type_fr_mouth_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_mouth_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_lower_neck_joint();
        const Type_fr_mouth_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_mouth_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_chassis_joint();
        const Type_fr_mouth_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_ear_link_COM : public TransformForce<Scalar, Type_fr_world_X_fr_right_ear_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_ear_link_COM();
        const Type_fr_world_X_fr_right_ear_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_world : public TransformForce<Scalar, Type_fr_right_ear_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_world();
        const Type_fr_right_ear_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_right_ear_joint : public TransformForce<Scalar, Type_fr_right_ear_link_COM_X_fr_right_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_right_ear_joint();
        const Type_fr_right_ear_link_COM_X_fr_right_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_head_joint : public TransformForce<Scalar, Type_fr_right_ear_link_COM_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_head_joint();
        const Type_fr_right_ear_link_COM_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_upper_neck_joint : public TransformForce<Scalar, Type_fr_right_ear_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_upper_neck_joint();
        const Type_fr_right_ear_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_right_ear_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_lower_neck_joint();
        const Type_fr_right_ear_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_right_ear_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_chassis_joint();
        const Type_fr_right_ear_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_wheel_link_COM : public TransformForce<Scalar, Type_fr_world_X_fr_right_wheel_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_wheel_link_COM();
        const Type_fr_world_X_fr_right_wheel_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_COM_X_fr_world : public TransformForce<Scalar, Type_fr_right_wheel_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_COM_X_fr_world();
        const Type_fr_right_wheel_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint : public TransformForce<Scalar, Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint();
        const Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_COM_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_right_wheel_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_COM_X_fr_chassis_joint();
        const Type_fr_right_wheel_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_neck_link_COM : public TransformForce<Scalar, Type_fr_world_X_fr_upper_neck_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_neck_link_COM();
        const Type_fr_world_X_fr_upper_neck_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_world : public TransformForce<Scalar, Type_fr_upper_neck_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_world();
        const Type_fr_upper_neck_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint : public TransformForce<Scalar, Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint();
        const Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint : public TransformForce<Scalar, Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint();
        const Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_upper_neck_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_chassis_joint();
        const Type_fr_upper_neck_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_tail_link_COM : public TransformForce<Scalar, Type_fr_world_X_fr_upper_tail_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_tail_link_COM();
        const Type_fr_world_X_fr_upper_tail_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_COM_X_fr_world : public TransformForce<Scalar, Type_fr_upper_tail_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_COM_X_fr_world();
        const Type_fr_upper_tail_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint : public TransformForce<Scalar, Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint();
        const Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_COM_X_fr_chassis_joint : public TransformForce<Scalar, Type_fr_upper_tail_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_COM_X_fr_chassis_joint();
        const Type_fr_upper_tail_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_lower_neck_link : public TransformForce<Scalar, Type_fr_upper_neck_link_X_fr_lower_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_lower_neck_link();
        const Type_fr_upper_neck_link_X_fr_lower_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_upper_neck_link : public TransformForce<Scalar, Type_fr_lower_neck_link_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_upper_neck_link();
        const Type_fr_lower_neck_link_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
public:
    ForceTransforms();
    void updateParameters();
    Type_fr_world_X_fr_chassis_link fr_world_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_world fr_chassis_link_X_fr_world;
    Type_fr_upper_neck_link_X_fr_head_link fr_upper_neck_link_X_fr_head_link;
    Type_fr_head_link_X_fr_upper_neck_link fr_head_link_X_fr_upper_neck_link;
    Type_fr_head_link_X_fr_left_ear_link fr_head_link_X_fr_left_ear_link;
    Type_fr_left_ear_link_X_fr_head_link fr_left_ear_link_X_fr_head_link;
    Type_fr_chassis_link_X_fr_left_wheel_link fr_chassis_link_X_fr_left_wheel_link;
    Type_fr_left_wheel_link_X_fr_chassis_link fr_left_wheel_link_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_lower_neck_link fr_chassis_link_X_fr_lower_neck_link;
    Type_fr_lower_neck_link_X_fr_chassis_link fr_lower_neck_link_X_fr_chassis_link;
    Type_fr_upper_tail_link_X_fr_lower_tail_link fr_upper_tail_link_X_fr_lower_tail_link;
    Type_fr_lower_tail_link_X_fr_upper_tail_link fr_lower_tail_link_X_fr_upper_tail_link;
    Type_fr_head_link_X_fr_mouth_link fr_head_link_X_fr_mouth_link;
    Type_fr_mouth_link_X_fr_head_link fr_mouth_link_X_fr_head_link;
    Type_fr_head_link_X_fr_right_ear_link fr_head_link_X_fr_right_ear_link;
    Type_fr_right_ear_link_X_fr_head_link fr_right_ear_link_X_fr_head_link;
    Type_fr_chassis_link_X_fr_right_wheel_link fr_chassis_link_X_fr_right_wheel_link;
    Type_fr_right_wheel_link_X_fr_chassis_link fr_right_wheel_link_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_upper_neck_link fr_chassis_link_X_fr_upper_neck_link;
    Type_fr_upper_neck_link_X_fr_chassis_link fr_upper_neck_link_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_upper_tail_link fr_chassis_link_X_fr_upper_tail_link;
    Type_fr_upper_tail_link_X_fr_chassis_link fr_upper_tail_link_X_fr_chassis_link;
    Type_fr_world_X_fr_chassis_link_COM fr_world_X_fr_chassis_link_COM;
    Type_fr_chassis_link_COM_X_fr_world fr_chassis_link_COM_X_fr_world;
    Type_fr_world_X_fr_fixed_base_link fr_world_X_fr_fixed_base_link;
    Type_fr_fixed_base_link_X_fr_world fr_fixed_base_link_X_fr_world;
    Type_fr_head_link_X_fr_head_camera_link fr_head_link_X_fr_head_camera_link;
    Type_fr_head_camera_link_X_fr_head_link fr_head_camera_link_X_fr_head_link;
    Type_fr_head_link_X_fr_head_link_COM fr_head_link_X_fr_head_link_COM;
    Type_fr_head_link_COM_X_fr_head_link fr_head_link_COM_X_fr_head_link;
    Type_fr_chassis_link_X_fr_imu_link fr_chassis_link_X_fr_imu_link;
    Type_fr_imu_link_X_fr_chassis_link fr_imu_link_X_fr_chassis_link;
    Type_fr_head_link_X_fr_left_ear_link_COM fr_head_link_X_fr_left_ear_link_COM;
    Type_fr_left_ear_link_COM_X_fr_head_link fr_left_ear_link_COM_X_fr_head_link;
    Type_fr_chassis_link_X_fr_left_wheel_link_COM fr_chassis_link_X_fr_left_wheel_link_COM;
    Type_fr_left_wheel_link_COM_X_fr_chassis_link fr_left_wheel_link_COM_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_lower_neck_link_COM fr_chassis_link_X_fr_lower_neck_link_COM;
    Type_fr_lower_neck_link_COM_X_fr_chassis_link fr_lower_neck_link_COM_X_fr_chassis_link;
    Type_fr_upper_tail_link_X_fr_lower_tail_link_COM fr_upper_tail_link_X_fr_lower_tail_link_COM;
    Type_fr_lower_tail_link_COM_X_fr_upper_tail_link fr_lower_tail_link_COM_X_fr_upper_tail_link;
    Type_fr_head_link_X_fr_mouth_link_COM fr_head_link_X_fr_mouth_link_COM;
    Type_fr_mouth_link_COM_X_fr_head_link fr_mouth_link_COM_X_fr_head_link;
    Type_fr_world_X_fr_parent_link fr_world_X_fr_parent_link;
    Type_fr_parent_link_X_fr_world fr_parent_link_X_fr_world;
    Type_fr_head_link_X_fr_right_ear_link_COM fr_head_link_X_fr_right_ear_link_COM;
    Type_fr_right_ear_link_COM_X_fr_head_link fr_right_ear_link_COM_X_fr_head_link;
    Type_fr_chassis_link_X_fr_right_wheel_link_COM fr_chassis_link_X_fr_right_wheel_link_COM;
    Type_fr_right_wheel_link_COM_X_fr_chassis_link fr_right_wheel_link_COM_X_fr_chassis_link;
    Type_fr_upper_neck_link_X_fr_upper_neck_link_COM fr_upper_neck_link_X_fr_upper_neck_link_COM;
    Type_fr_upper_neck_link_COM_X_fr_upper_neck_link fr_upper_neck_link_COM_X_fr_upper_neck_link;
    Type_fr_chassis_link_X_fr_upper_tail_link_COM fr_chassis_link_X_fr_upper_tail_link_COM;
    Type_fr_upper_tail_link_COM_X_fr_chassis_link fr_upper_tail_link_COM_X_fr_chassis_link;
    Type_fr_world_X_fr_world_COM fr_world_X_fr_world_COM;
    Type_fr_world_COM_X_fr_world fr_world_COM_X_fr_world;
    Type_fr_world_X_fr_chassis_joint fr_world_X_fr_chassis_joint;
    Type_fr_chassis_link_X_fr_chassis_joint fr_chassis_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_head_link fr_world_X_fr_head_link;
    Type_fr_world_X_fr_lower_neck_joint fr_world_X_fr_lower_neck_joint;
    Type_fr_world_X_fr_upper_neck_joint fr_world_X_fr_upper_neck_joint;
    Type_fr_world_X_fr_head_joint fr_world_X_fr_head_joint;
    Type_fr_head_link_X_fr_world fr_head_link_X_fr_world;
    Type_fr_head_link_X_fr_head_joint fr_head_link_X_fr_head_joint;
    Type_fr_head_link_X_fr_upper_neck_joint fr_head_link_X_fr_upper_neck_joint;
    Type_fr_head_link_X_fr_lower_neck_joint fr_head_link_X_fr_lower_neck_joint;
    Type_fr_head_link_X_fr_chassis_joint fr_head_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_left_ear_link fr_world_X_fr_left_ear_link;
    Type_fr_world_X_fr_left_ear_joint fr_world_X_fr_left_ear_joint;
    Type_fr_left_ear_link_X_fr_world fr_left_ear_link_X_fr_world;
    Type_fr_left_ear_link_X_fr_left_ear_joint fr_left_ear_link_X_fr_left_ear_joint;
    Type_fr_left_ear_link_X_fr_head_joint fr_left_ear_link_X_fr_head_joint;
    Type_fr_left_ear_link_X_fr_upper_neck_joint fr_left_ear_link_X_fr_upper_neck_joint;
    Type_fr_left_ear_link_X_fr_lower_neck_joint fr_left_ear_link_X_fr_lower_neck_joint;
    Type_fr_left_ear_link_X_fr_chassis_joint fr_left_ear_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_left_wheel_link fr_world_X_fr_left_wheel_link;
    Type_fr_world_X_fr_left_wheel_joint fr_world_X_fr_left_wheel_joint;
    Type_fr_left_wheel_link_X_fr_world fr_left_wheel_link_X_fr_world;
    Type_fr_left_wheel_link_X_fr_left_wheel_joint fr_left_wheel_link_X_fr_left_wheel_joint;
    Type_fr_left_wheel_link_X_fr_chassis_joint fr_left_wheel_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_lower_neck_link fr_world_X_fr_lower_neck_link;
    Type_fr_lower_neck_link_X_fr_world fr_lower_neck_link_X_fr_world;
    Type_fr_lower_neck_link_X_fr_lower_neck_joint fr_lower_neck_link_X_fr_lower_neck_joint;
    Type_fr_lower_neck_link_X_fr_chassis_joint fr_lower_neck_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_lower_tail_link fr_world_X_fr_lower_tail_link;
    Type_fr_world_X_fr_upper_tail_joint fr_world_X_fr_upper_tail_joint;
    Type_fr_world_X_fr_lower_tail_joint fr_world_X_fr_lower_tail_joint;
    Type_fr_lower_tail_link_X_fr_world fr_lower_tail_link_X_fr_world;
    Type_fr_lower_tail_link_X_fr_lower_tail_joint fr_lower_tail_link_X_fr_lower_tail_joint;
    Type_fr_lower_tail_link_X_fr_upper_tail_joint fr_lower_tail_link_X_fr_upper_tail_joint;
    Type_fr_lower_tail_link_X_fr_chassis_joint fr_lower_tail_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_mouth_link fr_world_X_fr_mouth_link;
    Type_fr_world_X_fr_mouth_joint fr_world_X_fr_mouth_joint;
    Type_fr_mouth_link_X_fr_world fr_mouth_link_X_fr_world;
    Type_fr_mouth_link_X_fr_mouth_joint fr_mouth_link_X_fr_mouth_joint;
    Type_fr_mouth_link_X_fr_head_joint fr_mouth_link_X_fr_head_joint;
    Type_fr_mouth_link_X_fr_upper_neck_joint fr_mouth_link_X_fr_upper_neck_joint;
    Type_fr_mouth_link_X_fr_lower_neck_joint fr_mouth_link_X_fr_lower_neck_joint;
    Type_fr_mouth_link_X_fr_chassis_joint fr_mouth_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_right_ear_link fr_world_X_fr_right_ear_link;
    Type_fr_world_X_fr_right_ear_joint fr_world_X_fr_right_ear_joint;
    Type_fr_right_ear_link_X_fr_world fr_right_ear_link_X_fr_world;
    Type_fr_right_ear_link_X_fr_right_ear_joint fr_right_ear_link_X_fr_right_ear_joint;
    Type_fr_right_ear_link_X_fr_head_joint fr_right_ear_link_X_fr_head_joint;
    Type_fr_right_ear_link_X_fr_upper_neck_joint fr_right_ear_link_X_fr_upper_neck_joint;
    Type_fr_right_ear_link_X_fr_lower_neck_joint fr_right_ear_link_X_fr_lower_neck_joint;
    Type_fr_right_ear_link_X_fr_chassis_joint fr_right_ear_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_right_wheel_link fr_world_X_fr_right_wheel_link;
    Type_fr_world_X_fr_right_wheel_joint fr_world_X_fr_right_wheel_joint;
    Type_fr_right_wheel_link_X_fr_world fr_right_wheel_link_X_fr_world;
    Type_fr_right_wheel_link_X_fr_right_wheel_joint fr_right_wheel_link_X_fr_right_wheel_joint;
    Type_fr_right_wheel_link_X_fr_chassis_joint fr_right_wheel_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_upper_neck_link fr_world_X_fr_upper_neck_link;
    Type_fr_upper_neck_link_X_fr_world fr_upper_neck_link_X_fr_world;
    Type_fr_upper_neck_link_X_fr_upper_neck_joint fr_upper_neck_link_X_fr_upper_neck_joint;
    Type_fr_upper_neck_link_X_fr_lower_neck_joint fr_upper_neck_link_X_fr_lower_neck_joint;
    Type_fr_upper_neck_link_X_fr_chassis_joint fr_upper_neck_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_upper_tail_link fr_world_X_fr_upper_tail_link;
    Type_fr_upper_tail_link_X_fr_world fr_upper_tail_link_X_fr_world;
    Type_fr_upper_tail_link_X_fr_upper_tail_joint fr_upper_tail_link_X_fr_upper_tail_joint;
    Type_fr_upper_tail_link_X_fr_chassis_joint fr_upper_tail_link_X_fr_chassis_joint;
    Type_fr_chassis_link_COM_X_fr_chassis_joint fr_chassis_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_head_camera_link fr_world_X_fr_head_camera_link;
    Type_fr_head_camera_link_X_fr_world fr_head_camera_link_X_fr_world;
    Type_fr_head_camera_link_X_fr_head_joint fr_head_camera_link_X_fr_head_joint;
    Type_fr_head_camera_link_X_fr_upper_neck_joint fr_head_camera_link_X_fr_upper_neck_joint;
    Type_fr_head_camera_link_X_fr_lower_neck_joint fr_head_camera_link_X_fr_lower_neck_joint;
    Type_fr_head_camera_link_X_fr_chassis_joint fr_head_camera_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_head_link_COM fr_world_X_fr_head_link_COM;
    Type_fr_head_link_COM_X_fr_world fr_head_link_COM_X_fr_world;
    Type_fr_head_link_COM_X_fr_head_joint fr_head_link_COM_X_fr_head_joint;
    Type_fr_head_link_COM_X_fr_upper_neck_joint fr_head_link_COM_X_fr_upper_neck_joint;
    Type_fr_head_link_COM_X_fr_lower_neck_joint fr_head_link_COM_X_fr_lower_neck_joint;
    Type_fr_head_link_COM_X_fr_chassis_joint fr_head_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_imu_link fr_world_X_fr_imu_link;
    Type_fr_imu_link_X_fr_world fr_imu_link_X_fr_world;
    Type_fr_imu_link_X_fr_chassis_joint fr_imu_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_left_ear_link_COM fr_world_X_fr_left_ear_link_COM;
    Type_fr_left_ear_link_COM_X_fr_world fr_left_ear_link_COM_X_fr_world;
    Type_fr_left_ear_link_COM_X_fr_left_ear_joint fr_left_ear_link_COM_X_fr_left_ear_joint;
    Type_fr_left_ear_link_COM_X_fr_head_joint fr_left_ear_link_COM_X_fr_head_joint;
    Type_fr_left_ear_link_COM_X_fr_upper_neck_joint fr_left_ear_link_COM_X_fr_upper_neck_joint;
    Type_fr_left_ear_link_COM_X_fr_lower_neck_joint fr_left_ear_link_COM_X_fr_lower_neck_joint;
    Type_fr_left_ear_link_COM_X_fr_chassis_joint fr_left_ear_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_left_wheel_link_COM fr_world_X_fr_left_wheel_link_COM;
    Type_fr_left_wheel_link_COM_X_fr_world fr_left_wheel_link_COM_X_fr_world;
    Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint fr_left_wheel_link_COM_X_fr_left_wheel_joint;
    Type_fr_left_wheel_link_COM_X_fr_chassis_joint fr_left_wheel_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_lower_neck_link_COM fr_world_X_fr_lower_neck_link_COM;
    Type_fr_lower_neck_link_COM_X_fr_world fr_lower_neck_link_COM_X_fr_world;
    Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint fr_lower_neck_link_COM_X_fr_lower_neck_joint;
    Type_fr_lower_neck_link_COM_X_fr_chassis_joint fr_lower_neck_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_lower_tail_link_COM fr_world_X_fr_lower_tail_link_COM;
    Type_fr_lower_tail_link_COM_X_fr_world fr_lower_tail_link_COM_X_fr_world;
    Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint fr_lower_tail_link_COM_X_fr_lower_tail_joint;
    Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint fr_lower_tail_link_COM_X_fr_upper_tail_joint;
    Type_fr_lower_tail_link_COM_X_fr_chassis_joint fr_lower_tail_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_mouth_link_COM fr_world_X_fr_mouth_link_COM;
    Type_fr_mouth_link_COM_X_fr_world fr_mouth_link_COM_X_fr_world;
    Type_fr_mouth_link_COM_X_fr_mouth_joint fr_mouth_link_COM_X_fr_mouth_joint;
    Type_fr_mouth_link_COM_X_fr_head_joint fr_mouth_link_COM_X_fr_head_joint;
    Type_fr_mouth_link_COM_X_fr_upper_neck_joint fr_mouth_link_COM_X_fr_upper_neck_joint;
    Type_fr_mouth_link_COM_X_fr_lower_neck_joint fr_mouth_link_COM_X_fr_lower_neck_joint;
    Type_fr_mouth_link_COM_X_fr_chassis_joint fr_mouth_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_right_ear_link_COM fr_world_X_fr_right_ear_link_COM;
    Type_fr_right_ear_link_COM_X_fr_world fr_right_ear_link_COM_X_fr_world;
    Type_fr_right_ear_link_COM_X_fr_right_ear_joint fr_right_ear_link_COM_X_fr_right_ear_joint;
    Type_fr_right_ear_link_COM_X_fr_head_joint fr_right_ear_link_COM_X_fr_head_joint;
    Type_fr_right_ear_link_COM_X_fr_upper_neck_joint fr_right_ear_link_COM_X_fr_upper_neck_joint;
    Type_fr_right_ear_link_COM_X_fr_lower_neck_joint fr_right_ear_link_COM_X_fr_lower_neck_joint;
    Type_fr_right_ear_link_COM_X_fr_chassis_joint fr_right_ear_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_right_wheel_link_COM fr_world_X_fr_right_wheel_link_COM;
    Type_fr_right_wheel_link_COM_X_fr_world fr_right_wheel_link_COM_X_fr_world;
    Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint fr_right_wheel_link_COM_X_fr_right_wheel_joint;
    Type_fr_right_wheel_link_COM_X_fr_chassis_joint fr_right_wheel_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_upper_neck_link_COM fr_world_X_fr_upper_neck_link_COM;
    Type_fr_upper_neck_link_COM_X_fr_world fr_upper_neck_link_COM_X_fr_world;
    Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint fr_upper_neck_link_COM_X_fr_upper_neck_joint;
    Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint fr_upper_neck_link_COM_X_fr_lower_neck_joint;
    Type_fr_upper_neck_link_COM_X_fr_chassis_joint fr_upper_neck_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_upper_tail_link_COM fr_world_X_fr_upper_tail_link_COM;
    Type_fr_upper_tail_link_COM_X_fr_world fr_upper_tail_link_COM_X_fr_world;
    Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint fr_upper_tail_link_COM_X_fr_upper_tail_joint;
    Type_fr_upper_tail_link_COM_X_fr_chassis_joint fr_upper_tail_link_COM_X_fr_chassis_joint;
    Type_fr_upper_neck_link_X_fr_lower_neck_link fr_upper_neck_link_X_fr_lower_neck_link;
    Type_fr_lower_neck_link_X_fr_upper_neck_link fr_lower_neck_link_X_fr_upper_neck_link;

protected:

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
template <typename TRAIT>
class HomogeneousTransforms {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef typename TRAIT::Scalar Scalar;

    typedef JointState<Scalar> JState;
    class Dummy {};
    typedef typename TransformHomogeneous<Scalar, Dummy>::MatrixType MatrixType;
public:
    class Type_fr_world_X_fr_chassis_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_chassis_link();
        const Type_fr_world_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_chassis_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_world();
        const Type_fr_chassis_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_head_link : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_head_link();
        const Type_fr_upper_neck_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_upper_neck_link : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_upper_neck_link();
        const Type_fr_head_link_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_left_ear_link : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_left_ear_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_left_ear_link();
        const Type_fr_head_link_X_fr_left_ear_link& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_head_link : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_head_link();
        const Type_fr_left_ear_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_left_wheel_link : public TransformHomogeneous<Scalar, Type_fr_chassis_link_X_fr_left_wheel_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_left_wheel_link();
        const Type_fr_chassis_link_X_fr_left_wheel_link& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_X_fr_chassis_link : public TransformHomogeneous<Scalar, Type_fr_left_wheel_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_X_fr_chassis_link();
        const Type_fr_left_wheel_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_lower_neck_link : public TransformHomogeneous<Scalar, Type_fr_chassis_link_X_fr_lower_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_lower_neck_link();
        const Type_fr_chassis_link_X_fr_lower_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_chassis_link : public TransformHomogeneous<Scalar, Type_fr_lower_neck_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_chassis_link();
        const Type_fr_lower_neck_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_lower_tail_link : public TransformHomogeneous<Scalar, Type_fr_upper_tail_link_X_fr_lower_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_lower_tail_link();
        const Type_fr_upper_tail_link_X_fr_lower_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_upper_tail_link : public TransformHomogeneous<Scalar, Type_fr_lower_tail_link_X_fr_upper_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_upper_tail_link();
        const Type_fr_lower_tail_link_X_fr_upper_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_mouth_link : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_mouth_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_mouth_link();
        const Type_fr_head_link_X_fr_mouth_link& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_head_link : public TransformHomogeneous<Scalar, Type_fr_mouth_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_head_link();
        const Type_fr_mouth_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_right_ear_link : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_right_ear_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_right_ear_link();
        const Type_fr_head_link_X_fr_right_ear_link& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_head_link : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_head_link();
        const Type_fr_right_ear_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_right_wheel_link : public TransformHomogeneous<Scalar, Type_fr_chassis_link_X_fr_right_wheel_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_right_wheel_link();
        const Type_fr_chassis_link_X_fr_right_wheel_link& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_X_fr_chassis_link : public TransformHomogeneous<Scalar, Type_fr_right_wheel_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_X_fr_chassis_link();
        const Type_fr_right_wheel_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_upper_neck_link : public TransformHomogeneous<Scalar, Type_fr_chassis_link_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_upper_neck_link();
        const Type_fr_chassis_link_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_chassis_link : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_chassis_link();
        const Type_fr_upper_neck_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_upper_tail_link : public TransformHomogeneous<Scalar, Type_fr_chassis_link_X_fr_upper_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_upper_tail_link();
        const Type_fr_chassis_link_X_fr_upper_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_chassis_link : public TransformHomogeneous<Scalar, Type_fr_upper_tail_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_chassis_link();
        const Type_fr_upper_tail_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_chassis_link_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_chassis_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_chassis_link_COM();
        const Type_fr_world_X_fr_chassis_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_chassis_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_COM_X_fr_world();
        const Type_fr_chassis_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_fixed_base_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_fixed_base_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_fixed_base_link();
        const Type_fr_world_X_fr_fixed_base_link& update(const JState&);
    protected:
    };
    
    class Type_fr_fixed_base_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_fixed_base_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_fixed_base_link_X_fr_world();
        const Type_fr_fixed_base_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_head_camera_link : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_head_camera_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_head_camera_link();
        const Type_fr_head_link_X_fr_head_camera_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_head_link : public TransformHomogeneous<Scalar, Type_fr_head_camera_link_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_head_link();
        const Type_fr_head_camera_link_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_head_link_COM : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_head_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_head_link_COM();
        const Type_fr_head_link_X_fr_head_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_head_link : public TransformHomogeneous<Scalar, Type_fr_head_link_COM_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_head_link();
        const Type_fr_head_link_COM_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_imu_link : public TransformHomogeneous<Scalar, Type_fr_chassis_link_X_fr_imu_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_imu_link();
        const Type_fr_chassis_link_X_fr_imu_link& update(const JState&);
    protected:
    };
    
    class Type_fr_imu_link_X_fr_chassis_link : public TransformHomogeneous<Scalar, Type_fr_imu_link_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_imu_link_X_fr_chassis_link();
        const Type_fr_imu_link_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_left_ear_link_COM : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_left_ear_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_left_ear_link_COM();
        const Type_fr_head_link_X_fr_left_ear_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_head_link : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_COM_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_head_link();
        const Type_fr_left_ear_link_COM_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_left_wheel_link_COM : public TransformHomogeneous<Scalar, Type_fr_chassis_link_X_fr_left_wheel_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_left_wheel_link_COM();
        const Type_fr_chassis_link_X_fr_left_wheel_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_COM_X_fr_chassis_link : public TransformHomogeneous<Scalar, Type_fr_left_wheel_link_COM_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_COM_X_fr_chassis_link();
        const Type_fr_left_wheel_link_COM_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_lower_neck_link_COM : public TransformHomogeneous<Scalar, Type_fr_chassis_link_X_fr_lower_neck_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_lower_neck_link_COM();
        const Type_fr_chassis_link_X_fr_lower_neck_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_COM_X_fr_chassis_link : public TransformHomogeneous<Scalar, Type_fr_lower_neck_link_COM_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_COM_X_fr_chassis_link();
        const Type_fr_lower_neck_link_COM_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_lower_tail_link_COM : public TransformHomogeneous<Scalar, Type_fr_upper_tail_link_X_fr_lower_tail_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_lower_tail_link_COM();
        const Type_fr_upper_tail_link_X_fr_lower_tail_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_upper_tail_link : public TransformHomogeneous<Scalar, Type_fr_lower_tail_link_COM_X_fr_upper_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_upper_tail_link();
        const Type_fr_lower_tail_link_COM_X_fr_upper_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_mouth_link_COM : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_mouth_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_mouth_link_COM();
        const Type_fr_head_link_X_fr_mouth_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_head_link : public TransformHomogeneous<Scalar, Type_fr_mouth_link_COM_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_head_link();
        const Type_fr_mouth_link_COM_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_parent_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_parent_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_parent_link();
        const Type_fr_world_X_fr_parent_link& update(const JState&);
    protected:
    };
    
    class Type_fr_parent_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_parent_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_parent_link_X_fr_world();
        const Type_fr_parent_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_right_ear_link_COM : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_right_ear_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_right_ear_link_COM();
        const Type_fr_head_link_X_fr_right_ear_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_head_link : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_COM_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_head_link();
        const Type_fr_right_ear_link_COM_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_right_wheel_link_COM : public TransformHomogeneous<Scalar, Type_fr_chassis_link_X_fr_right_wheel_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_right_wheel_link_COM();
        const Type_fr_chassis_link_X_fr_right_wheel_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_COM_X_fr_chassis_link : public TransformHomogeneous<Scalar, Type_fr_right_wheel_link_COM_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_COM_X_fr_chassis_link();
        const Type_fr_right_wheel_link_COM_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_upper_neck_link_COM : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_X_fr_upper_neck_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_upper_neck_link_COM();
        const Type_fr_upper_neck_link_X_fr_upper_neck_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_upper_neck_link : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_COM_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_upper_neck_link();
        const Type_fr_upper_neck_link_COM_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_upper_tail_link_COM : public TransformHomogeneous<Scalar, Type_fr_chassis_link_X_fr_upper_tail_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_upper_tail_link_COM();
        const Type_fr_chassis_link_X_fr_upper_tail_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_COM_X_fr_chassis_link : public TransformHomogeneous<Scalar, Type_fr_upper_tail_link_COM_X_fr_chassis_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_COM_X_fr_chassis_link();
        const Type_fr_upper_tail_link_COM_X_fr_chassis_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_world_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_world_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_world_COM();
        const Type_fr_world_X_fr_world_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_world_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_world_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_COM_X_fr_world();
        const Type_fr_world_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_chassis_joint();
        const Type_fr_world_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_chassis_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_X_fr_chassis_joint();
        const Type_fr_chassis_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_head_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_head_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_head_link();
        const Type_fr_world_X_fr_head_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_neck_joint();
        const Type_fr_world_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_neck_joint : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_neck_joint();
        const Type_fr_world_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_head_joint : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_head_joint();
        const Type_fr_world_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_world();
        const Type_fr_head_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_head_joint : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_head_joint();
        const Type_fr_head_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_upper_neck_joint : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_upper_neck_joint();
        const Type_fr_head_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_lower_neck_joint();
        const Type_fr_head_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_head_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_X_fr_chassis_joint();
        const Type_fr_head_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_ear_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_left_ear_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_ear_link();
        const Type_fr_world_X_fr_left_ear_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_ear_joint : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_left_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_ear_joint();
        const Type_fr_world_X_fr_left_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_world();
        const Type_fr_left_ear_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_left_ear_joint : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_X_fr_left_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_left_ear_joint();
        const Type_fr_left_ear_link_X_fr_left_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_head_joint : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_head_joint();
        const Type_fr_left_ear_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_upper_neck_joint : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_upper_neck_joint();
        const Type_fr_left_ear_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_lower_neck_joint();
        const Type_fr_left_ear_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_X_fr_chassis_joint();
        const Type_fr_left_ear_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_wheel_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_left_wheel_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_wheel_link();
        const Type_fr_world_X_fr_left_wheel_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_wheel_joint : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_left_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_wheel_joint();
        const Type_fr_world_X_fr_left_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_left_wheel_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_X_fr_world();
        const Type_fr_left_wheel_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_X_fr_left_wheel_joint : public TransformHomogeneous<Scalar, Type_fr_left_wheel_link_X_fr_left_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_X_fr_left_wheel_joint();
        const Type_fr_left_wheel_link_X_fr_left_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_left_wheel_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_X_fr_chassis_joint();
        const Type_fr_left_wheel_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_neck_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_lower_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_neck_link();
        const Type_fr_world_X_fr_lower_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_lower_neck_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_world();
        const Type_fr_lower_neck_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_lower_neck_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_lower_neck_joint();
        const Type_fr_lower_neck_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_lower_neck_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_chassis_joint();
        const Type_fr_lower_neck_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_tail_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_lower_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_tail_link();
        const Type_fr_world_X_fr_lower_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_tail_joint : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_tail_joint();
        const Type_fr_world_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_tail_joint : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_lower_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_tail_joint();
        const Type_fr_world_X_fr_lower_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_lower_tail_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_world();
        const Type_fr_lower_tail_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_lower_tail_joint : public TransformHomogeneous<Scalar, Type_fr_lower_tail_link_X_fr_lower_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_lower_tail_joint();
        const Type_fr_lower_tail_link_X_fr_lower_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_upper_tail_joint : public TransformHomogeneous<Scalar, Type_fr_lower_tail_link_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_upper_tail_joint();
        const Type_fr_lower_tail_link_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_lower_tail_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_X_fr_chassis_joint();
        const Type_fr_lower_tail_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_mouth_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_mouth_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_mouth_link();
        const Type_fr_world_X_fr_mouth_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_mouth_joint : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_mouth_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_mouth_joint();
        const Type_fr_world_X_fr_mouth_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_mouth_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_world();
        const Type_fr_mouth_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_mouth_joint : public TransformHomogeneous<Scalar, Type_fr_mouth_link_X_fr_mouth_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_mouth_joint();
        const Type_fr_mouth_link_X_fr_mouth_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_head_joint : public TransformHomogeneous<Scalar, Type_fr_mouth_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_head_joint();
        const Type_fr_mouth_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_upper_neck_joint : public TransformHomogeneous<Scalar, Type_fr_mouth_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_upper_neck_joint();
        const Type_fr_mouth_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_mouth_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_lower_neck_joint();
        const Type_fr_mouth_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_mouth_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_X_fr_chassis_joint();
        const Type_fr_mouth_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_ear_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_right_ear_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_ear_link();
        const Type_fr_world_X_fr_right_ear_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_ear_joint : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_right_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_ear_joint();
        const Type_fr_world_X_fr_right_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_world();
        const Type_fr_right_ear_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_right_ear_joint : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_X_fr_right_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_right_ear_joint();
        const Type_fr_right_ear_link_X_fr_right_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_head_joint : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_head_joint();
        const Type_fr_right_ear_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_upper_neck_joint : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_upper_neck_joint();
        const Type_fr_right_ear_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_lower_neck_joint();
        const Type_fr_right_ear_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_X_fr_chassis_joint();
        const Type_fr_right_ear_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_wheel_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_right_wheel_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_wheel_link();
        const Type_fr_world_X_fr_right_wheel_link& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_wheel_joint : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_right_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_wheel_joint();
        const Type_fr_world_X_fr_right_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_right_wheel_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_X_fr_world();
        const Type_fr_right_wheel_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_X_fr_right_wheel_joint : public TransformHomogeneous<Scalar, Type_fr_right_wheel_link_X_fr_right_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_X_fr_right_wheel_joint();
        const Type_fr_right_wheel_link_X_fr_right_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_right_wheel_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_X_fr_chassis_joint();
        const Type_fr_right_wheel_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_neck_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_neck_link();
        const Type_fr_world_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_world();
        const Type_fr_upper_neck_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_upper_neck_joint : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_upper_neck_joint();
        const Type_fr_upper_neck_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_lower_neck_joint();
        const Type_fr_upper_neck_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_chassis_joint();
        const Type_fr_upper_neck_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_tail_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_upper_tail_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_tail_link();
        const Type_fr_world_X_fr_upper_tail_link& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_upper_tail_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_world();
        const Type_fr_upper_tail_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_upper_tail_joint : public TransformHomogeneous<Scalar, Type_fr_upper_tail_link_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_upper_tail_joint();
        const Type_fr_upper_tail_link_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_upper_tail_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_X_fr_chassis_joint();
        const Type_fr_upper_tail_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_chassis_link_COM_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_chassis_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_chassis_link_COM_X_fr_chassis_joint();
        const Type_fr_chassis_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_head_camera_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_head_camera_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_head_camera_link();
        const Type_fr_world_X_fr_head_camera_link& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_head_camera_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_world();
        const Type_fr_head_camera_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_head_joint : public TransformHomogeneous<Scalar, Type_fr_head_camera_link_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_head_joint();
        const Type_fr_head_camera_link_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_upper_neck_joint : public TransformHomogeneous<Scalar, Type_fr_head_camera_link_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_upper_neck_joint();
        const Type_fr_head_camera_link_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_head_camera_link_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_lower_neck_joint();
        const Type_fr_head_camera_link_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_camera_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_head_camera_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_camera_link_X_fr_chassis_joint();
        const Type_fr_head_camera_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_head_link_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_head_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_head_link_COM();
        const Type_fr_world_X_fr_head_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_head_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_world();
        const Type_fr_head_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_head_joint : public TransformHomogeneous<Scalar, Type_fr_head_link_COM_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_head_joint();
        const Type_fr_head_link_COM_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_upper_neck_joint : public TransformHomogeneous<Scalar, Type_fr_head_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_upper_neck_joint();
        const Type_fr_head_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_head_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_lower_neck_joint();
        const Type_fr_head_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_head_link_COM_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_head_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_head_link_COM_X_fr_chassis_joint();
        const Type_fr_head_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_imu_link : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_imu_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_imu_link();
        const Type_fr_world_X_fr_imu_link& update(const JState&);
    protected:
    };
    
    class Type_fr_imu_link_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_imu_link_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_imu_link_X_fr_world();
        const Type_fr_imu_link_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_imu_link_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_imu_link_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_imu_link_X_fr_chassis_joint();
        const Type_fr_imu_link_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_ear_link_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_left_ear_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_ear_link_COM();
        const Type_fr_world_X_fr_left_ear_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_world();
        const Type_fr_left_ear_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_left_ear_joint : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_COM_X_fr_left_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_left_ear_joint();
        const Type_fr_left_ear_link_COM_X_fr_left_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_head_joint : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_COM_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_head_joint();
        const Type_fr_left_ear_link_COM_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_upper_neck_joint : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_upper_neck_joint();
        const Type_fr_left_ear_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_lower_neck_joint();
        const Type_fr_left_ear_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_ear_link_COM_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_left_ear_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_ear_link_COM_X_fr_chassis_joint();
        const Type_fr_left_ear_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_left_wheel_link_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_left_wheel_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_left_wheel_link_COM();
        const Type_fr_world_X_fr_left_wheel_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_left_wheel_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_COM_X_fr_world();
        const Type_fr_left_wheel_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint : public TransformHomogeneous<Scalar, Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint();
        const Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_left_wheel_link_COM_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_left_wheel_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_left_wheel_link_COM_X_fr_chassis_joint();
        const Type_fr_left_wheel_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_neck_link_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_lower_neck_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_neck_link_COM();
        const Type_fr_world_X_fr_lower_neck_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_lower_neck_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_COM_X_fr_world();
        const Type_fr_lower_neck_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint();
        const Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_COM_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_lower_neck_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_COM_X_fr_chassis_joint();
        const Type_fr_lower_neck_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_lower_tail_link_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_lower_tail_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_lower_tail_link_COM();
        const Type_fr_world_X_fr_lower_tail_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_lower_tail_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_world();
        const Type_fr_lower_tail_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint : public TransformHomogeneous<Scalar, Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint();
        const Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint : public TransformHomogeneous<Scalar, Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint();
        const Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_tail_link_COM_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_lower_tail_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_tail_link_COM_X_fr_chassis_joint();
        const Type_fr_lower_tail_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_mouth_link_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_mouth_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_mouth_link_COM();
        const Type_fr_world_X_fr_mouth_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_mouth_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_world();
        const Type_fr_mouth_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_mouth_joint : public TransformHomogeneous<Scalar, Type_fr_mouth_link_COM_X_fr_mouth_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_mouth_joint();
        const Type_fr_mouth_link_COM_X_fr_mouth_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_head_joint : public TransformHomogeneous<Scalar, Type_fr_mouth_link_COM_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_head_joint();
        const Type_fr_mouth_link_COM_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_upper_neck_joint : public TransformHomogeneous<Scalar, Type_fr_mouth_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_upper_neck_joint();
        const Type_fr_mouth_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_mouth_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_lower_neck_joint();
        const Type_fr_mouth_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_mouth_link_COM_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_mouth_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_mouth_link_COM_X_fr_chassis_joint();
        const Type_fr_mouth_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_ear_link_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_right_ear_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_ear_link_COM();
        const Type_fr_world_X_fr_right_ear_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_world();
        const Type_fr_right_ear_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_right_ear_joint : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_COM_X_fr_right_ear_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_right_ear_joint();
        const Type_fr_right_ear_link_COM_X_fr_right_ear_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_head_joint : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_COM_X_fr_head_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_head_joint();
        const Type_fr_right_ear_link_COM_X_fr_head_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_upper_neck_joint : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_upper_neck_joint();
        const Type_fr_right_ear_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_lower_neck_joint();
        const Type_fr_right_ear_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_ear_link_COM_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_right_ear_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_ear_link_COM_X_fr_chassis_joint();
        const Type_fr_right_ear_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_right_wheel_link_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_right_wheel_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_right_wheel_link_COM();
        const Type_fr_world_X_fr_right_wheel_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_right_wheel_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_COM_X_fr_world();
        const Type_fr_right_wheel_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint : public TransformHomogeneous<Scalar, Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint();
        const Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_right_wheel_link_COM_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_right_wheel_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_right_wheel_link_COM_X_fr_chassis_joint();
        const Type_fr_right_wheel_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_neck_link_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_upper_neck_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_neck_link_COM();
        const Type_fr_world_X_fr_upper_neck_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_world();
        const Type_fr_upper_neck_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint();
        const Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint();
        const Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_COM_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_COM_X_fr_chassis_joint();
        const Type_fr_upper_neck_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_world_X_fr_upper_tail_link_COM : public TransformHomogeneous<Scalar, Type_fr_world_X_fr_upper_tail_link_COM>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_world_X_fr_upper_tail_link_COM();
        const Type_fr_world_X_fr_upper_tail_link_COM& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_COM_X_fr_world : public TransformHomogeneous<Scalar, Type_fr_upper_tail_link_COM_X_fr_world>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_COM_X_fr_world();
        const Type_fr_upper_tail_link_COM_X_fr_world& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint : public TransformHomogeneous<Scalar, Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint();
        const Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_tail_link_COM_X_fr_chassis_joint : public TransformHomogeneous<Scalar, Type_fr_upper_tail_link_COM_X_fr_chassis_joint>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_tail_link_COM_X_fr_chassis_joint();
        const Type_fr_upper_tail_link_COM_X_fr_chassis_joint& update(const JState&);
    protected:
    };
    
    class Type_fr_upper_neck_link_X_fr_lower_neck_link : public TransformHomogeneous<Scalar, Type_fr_upper_neck_link_X_fr_lower_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_upper_neck_link_X_fr_lower_neck_link();
        const Type_fr_upper_neck_link_X_fr_lower_neck_link& update(const JState&);
    protected:
    };
    
    class Type_fr_lower_neck_link_X_fr_upper_neck_link : public TransformHomogeneous<Scalar, Type_fr_lower_neck_link_X_fr_upper_neck_link>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Type_fr_lower_neck_link_X_fr_upper_neck_link();
        const Type_fr_lower_neck_link_X_fr_upper_neck_link& update(const JState&);
    protected:
    };
    
public:
    HomogeneousTransforms();
    void updateParameters();
    Type_fr_world_X_fr_chassis_link fr_world_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_world fr_chassis_link_X_fr_world;
    Type_fr_upper_neck_link_X_fr_head_link fr_upper_neck_link_X_fr_head_link;
    Type_fr_head_link_X_fr_upper_neck_link fr_head_link_X_fr_upper_neck_link;
    Type_fr_head_link_X_fr_left_ear_link fr_head_link_X_fr_left_ear_link;
    Type_fr_left_ear_link_X_fr_head_link fr_left_ear_link_X_fr_head_link;
    Type_fr_chassis_link_X_fr_left_wheel_link fr_chassis_link_X_fr_left_wheel_link;
    Type_fr_left_wheel_link_X_fr_chassis_link fr_left_wheel_link_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_lower_neck_link fr_chassis_link_X_fr_lower_neck_link;
    Type_fr_lower_neck_link_X_fr_chassis_link fr_lower_neck_link_X_fr_chassis_link;
    Type_fr_upper_tail_link_X_fr_lower_tail_link fr_upper_tail_link_X_fr_lower_tail_link;
    Type_fr_lower_tail_link_X_fr_upper_tail_link fr_lower_tail_link_X_fr_upper_tail_link;
    Type_fr_head_link_X_fr_mouth_link fr_head_link_X_fr_mouth_link;
    Type_fr_mouth_link_X_fr_head_link fr_mouth_link_X_fr_head_link;
    Type_fr_head_link_X_fr_right_ear_link fr_head_link_X_fr_right_ear_link;
    Type_fr_right_ear_link_X_fr_head_link fr_right_ear_link_X_fr_head_link;
    Type_fr_chassis_link_X_fr_right_wheel_link fr_chassis_link_X_fr_right_wheel_link;
    Type_fr_right_wheel_link_X_fr_chassis_link fr_right_wheel_link_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_upper_neck_link fr_chassis_link_X_fr_upper_neck_link;
    Type_fr_upper_neck_link_X_fr_chassis_link fr_upper_neck_link_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_upper_tail_link fr_chassis_link_X_fr_upper_tail_link;
    Type_fr_upper_tail_link_X_fr_chassis_link fr_upper_tail_link_X_fr_chassis_link;
    Type_fr_world_X_fr_chassis_link_COM fr_world_X_fr_chassis_link_COM;
    Type_fr_chassis_link_COM_X_fr_world fr_chassis_link_COM_X_fr_world;
    Type_fr_world_X_fr_fixed_base_link fr_world_X_fr_fixed_base_link;
    Type_fr_fixed_base_link_X_fr_world fr_fixed_base_link_X_fr_world;
    Type_fr_head_link_X_fr_head_camera_link fr_head_link_X_fr_head_camera_link;
    Type_fr_head_camera_link_X_fr_head_link fr_head_camera_link_X_fr_head_link;
    Type_fr_head_link_X_fr_head_link_COM fr_head_link_X_fr_head_link_COM;
    Type_fr_head_link_COM_X_fr_head_link fr_head_link_COM_X_fr_head_link;
    Type_fr_chassis_link_X_fr_imu_link fr_chassis_link_X_fr_imu_link;
    Type_fr_imu_link_X_fr_chassis_link fr_imu_link_X_fr_chassis_link;
    Type_fr_head_link_X_fr_left_ear_link_COM fr_head_link_X_fr_left_ear_link_COM;
    Type_fr_left_ear_link_COM_X_fr_head_link fr_left_ear_link_COM_X_fr_head_link;
    Type_fr_chassis_link_X_fr_left_wheel_link_COM fr_chassis_link_X_fr_left_wheel_link_COM;
    Type_fr_left_wheel_link_COM_X_fr_chassis_link fr_left_wheel_link_COM_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_lower_neck_link_COM fr_chassis_link_X_fr_lower_neck_link_COM;
    Type_fr_lower_neck_link_COM_X_fr_chassis_link fr_lower_neck_link_COM_X_fr_chassis_link;
    Type_fr_upper_tail_link_X_fr_lower_tail_link_COM fr_upper_tail_link_X_fr_lower_tail_link_COM;
    Type_fr_lower_tail_link_COM_X_fr_upper_tail_link fr_lower_tail_link_COM_X_fr_upper_tail_link;
    Type_fr_head_link_X_fr_mouth_link_COM fr_head_link_X_fr_mouth_link_COM;
    Type_fr_mouth_link_COM_X_fr_head_link fr_mouth_link_COM_X_fr_head_link;
    Type_fr_world_X_fr_parent_link fr_world_X_fr_parent_link;
    Type_fr_parent_link_X_fr_world fr_parent_link_X_fr_world;
    Type_fr_head_link_X_fr_right_ear_link_COM fr_head_link_X_fr_right_ear_link_COM;
    Type_fr_right_ear_link_COM_X_fr_head_link fr_right_ear_link_COM_X_fr_head_link;
    Type_fr_chassis_link_X_fr_right_wheel_link_COM fr_chassis_link_X_fr_right_wheel_link_COM;
    Type_fr_right_wheel_link_COM_X_fr_chassis_link fr_right_wheel_link_COM_X_fr_chassis_link;
    Type_fr_upper_neck_link_X_fr_upper_neck_link_COM fr_upper_neck_link_X_fr_upper_neck_link_COM;
    Type_fr_upper_neck_link_COM_X_fr_upper_neck_link fr_upper_neck_link_COM_X_fr_upper_neck_link;
    Type_fr_chassis_link_X_fr_upper_tail_link_COM fr_chassis_link_X_fr_upper_tail_link_COM;
    Type_fr_upper_tail_link_COM_X_fr_chassis_link fr_upper_tail_link_COM_X_fr_chassis_link;
    Type_fr_world_X_fr_world_COM fr_world_X_fr_world_COM;
    Type_fr_world_COM_X_fr_world fr_world_COM_X_fr_world;
    Type_fr_world_X_fr_chassis_joint fr_world_X_fr_chassis_joint;
    Type_fr_chassis_link_X_fr_chassis_joint fr_chassis_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_head_link fr_world_X_fr_head_link;
    Type_fr_world_X_fr_lower_neck_joint fr_world_X_fr_lower_neck_joint;
    Type_fr_world_X_fr_upper_neck_joint fr_world_X_fr_upper_neck_joint;
    Type_fr_world_X_fr_head_joint fr_world_X_fr_head_joint;
    Type_fr_head_link_X_fr_world fr_head_link_X_fr_world;
    Type_fr_head_link_X_fr_head_joint fr_head_link_X_fr_head_joint;
    Type_fr_head_link_X_fr_upper_neck_joint fr_head_link_X_fr_upper_neck_joint;
    Type_fr_head_link_X_fr_lower_neck_joint fr_head_link_X_fr_lower_neck_joint;
    Type_fr_head_link_X_fr_chassis_joint fr_head_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_left_ear_link fr_world_X_fr_left_ear_link;
    Type_fr_world_X_fr_left_ear_joint fr_world_X_fr_left_ear_joint;
    Type_fr_left_ear_link_X_fr_world fr_left_ear_link_X_fr_world;
    Type_fr_left_ear_link_X_fr_left_ear_joint fr_left_ear_link_X_fr_left_ear_joint;
    Type_fr_left_ear_link_X_fr_head_joint fr_left_ear_link_X_fr_head_joint;
    Type_fr_left_ear_link_X_fr_upper_neck_joint fr_left_ear_link_X_fr_upper_neck_joint;
    Type_fr_left_ear_link_X_fr_lower_neck_joint fr_left_ear_link_X_fr_lower_neck_joint;
    Type_fr_left_ear_link_X_fr_chassis_joint fr_left_ear_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_left_wheel_link fr_world_X_fr_left_wheel_link;
    Type_fr_world_X_fr_left_wheel_joint fr_world_X_fr_left_wheel_joint;
    Type_fr_left_wheel_link_X_fr_world fr_left_wheel_link_X_fr_world;
    Type_fr_left_wheel_link_X_fr_left_wheel_joint fr_left_wheel_link_X_fr_left_wheel_joint;
    Type_fr_left_wheel_link_X_fr_chassis_joint fr_left_wheel_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_lower_neck_link fr_world_X_fr_lower_neck_link;
    Type_fr_lower_neck_link_X_fr_world fr_lower_neck_link_X_fr_world;
    Type_fr_lower_neck_link_X_fr_lower_neck_joint fr_lower_neck_link_X_fr_lower_neck_joint;
    Type_fr_lower_neck_link_X_fr_chassis_joint fr_lower_neck_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_lower_tail_link fr_world_X_fr_lower_tail_link;
    Type_fr_world_X_fr_upper_tail_joint fr_world_X_fr_upper_tail_joint;
    Type_fr_world_X_fr_lower_tail_joint fr_world_X_fr_lower_tail_joint;
    Type_fr_lower_tail_link_X_fr_world fr_lower_tail_link_X_fr_world;
    Type_fr_lower_tail_link_X_fr_lower_tail_joint fr_lower_tail_link_X_fr_lower_tail_joint;
    Type_fr_lower_tail_link_X_fr_upper_tail_joint fr_lower_tail_link_X_fr_upper_tail_joint;
    Type_fr_lower_tail_link_X_fr_chassis_joint fr_lower_tail_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_mouth_link fr_world_X_fr_mouth_link;
    Type_fr_world_X_fr_mouth_joint fr_world_X_fr_mouth_joint;
    Type_fr_mouth_link_X_fr_world fr_mouth_link_X_fr_world;
    Type_fr_mouth_link_X_fr_mouth_joint fr_mouth_link_X_fr_mouth_joint;
    Type_fr_mouth_link_X_fr_head_joint fr_mouth_link_X_fr_head_joint;
    Type_fr_mouth_link_X_fr_upper_neck_joint fr_mouth_link_X_fr_upper_neck_joint;
    Type_fr_mouth_link_X_fr_lower_neck_joint fr_mouth_link_X_fr_lower_neck_joint;
    Type_fr_mouth_link_X_fr_chassis_joint fr_mouth_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_right_ear_link fr_world_X_fr_right_ear_link;
    Type_fr_world_X_fr_right_ear_joint fr_world_X_fr_right_ear_joint;
    Type_fr_right_ear_link_X_fr_world fr_right_ear_link_X_fr_world;
    Type_fr_right_ear_link_X_fr_right_ear_joint fr_right_ear_link_X_fr_right_ear_joint;
    Type_fr_right_ear_link_X_fr_head_joint fr_right_ear_link_X_fr_head_joint;
    Type_fr_right_ear_link_X_fr_upper_neck_joint fr_right_ear_link_X_fr_upper_neck_joint;
    Type_fr_right_ear_link_X_fr_lower_neck_joint fr_right_ear_link_X_fr_lower_neck_joint;
    Type_fr_right_ear_link_X_fr_chassis_joint fr_right_ear_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_right_wheel_link fr_world_X_fr_right_wheel_link;
    Type_fr_world_X_fr_right_wheel_joint fr_world_X_fr_right_wheel_joint;
    Type_fr_right_wheel_link_X_fr_world fr_right_wheel_link_X_fr_world;
    Type_fr_right_wheel_link_X_fr_right_wheel_joint fr_right_wheel_link_X_fr_right_wheel_joint;
    Type_fr_right_wheel_link_X_fr_chassis_joint fr_right_wheel_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_upper_neck_link fr_world_X_fr_upper_neck_link;
    Type_fr_upper_neck_link_X_fr_world fr_upper_neck_link_X_fr_world;
    Type_fr_upper_neck_link_X_fr_upper_neck_joint fr_upper_neck_link_X_fr_upper_neck_joint;
    Type_fr_upper_neck_link_X_fr_lower_neck_joint fr_upper_neck_link_X_fr_lower_neck_joint;
    Type_fr_upper_neck_link_X_fr_chassis_joint fr_upper_neck_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_upper_tail_link fr_world_X_fr_upper_tail_link;
    Type_fr_upper_tail_link_X_fr_world fr_upper_tail_link_X_fr_world;
    Type_fr_upper_tail_link_X_fr_upper_tail_joint fr_upper_tail_link_X_fr_upper_tail_joint;
    Type_fr_upper_tail_link_X_fr_chassis_joint fr_upper_tail_link_X_fr_chassis_joint;
    Type_fr_chassis_link_COM_X_fr_chassis_joint fr_chassis_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_head_camera_link fr_world_X_fr_head_camera_link;
    Type_fr_head_camera_link_X_fr_world fr_head_camera_link_X_fr_world;
    Type_fr_head_camera_link_X_fr_head_joint fr_head_camera_link_X_fr_head_joint;
    Type_fr_head_camera_link_X_fr_upper_neck_joint fr_head_camera_link_X_fr_upper_neck_joint;
    Type_fr_head_camera_link_X_fr_lower_neck_joint fr_head_camera_link_X_fr_lower_neck_joint;
    Type_fr_head_camera_link_X_fr_chassis_joint fr_head_camera_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_head_link_COM fr_world_X_fr_head_link_COM;
    Type_fr_head_link_COM_X_fr_world fr_head_link_COM_X_fr_world;
    Type_fr_head_link_COM_X_fr_head_joint fr_head_link_COM_X_fr_head_joint;
    Type_fr_head_link_COM_X_fr_upper_neck_joint fr_head_link_COM_X_fr_upper_neck_joint;
    Type_fr_head_link_COM_X_fr_lower_neck_joint fr_head_link_COM_X_fr_lower_neck_joint;
    Type_fr_head_link_COM_X_fr_chassis_joint fr_head_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_imu_link fr_world_X_fr_imu_link;
    Type_fr_imu_link_X_fr_world fr_imu_link_X_fr_world;
    Type_fr_imu_link_X_fr_chassis_joint fr_imu_link_X_fr_chassis_joint;
    Type_fr_world_X_fr_left_ear_link_COM fr_world_X_fr_left_ear_link_COM;
    Type_fr_left_ear_link_COM_X_fr_world fr_left_ear_link_COM_X_fr_world;
    Type_fr_left_ear_link_COM_X_fr_left_ear_joint fr_left_ear_link_COM_X_fr_left_ear_joint;
    Type_fr_left_ear_link_COM_X_fr_head_joint fr_left_ear_link_COM_X_fr_head_joint;
    Type_fr_left_ear_link_COM_X_fr_upper_neck_joint fr_left_ear_link_COM_X_fr_upper_neck_joint;
    Type_fr_left_ear_link_COM_X_fr_lower_neck_joint fr_left_ear_link_COM_X_fr_lower_neck_joint;
    Type_fr_left_ear_link_COM_X_fr_chassis_joint fr_left_ear_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_left_wheel_link_COM fr_world_X_fr_left_wheel_link_COM;
    Type_fr_left_wheel_link_COM_X_fr_world fr_left_wheel_link_COM_X_fr_world;
    Type_fr_left_wheel_link_COM_X_fr_left_wheel_joint fr_left_wheel_link_COM_X_fr_left_wheel_joint;
    Type_fr_left_wheel_link_COM_X_fr_chassis_joint fr_left_wheel_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_lower_neck_link_COM fr_world_X_fr_lower_neck_link_COM;
    Type_fr_lower_neck_link_COM_X_fr_world fr_lower_neck_link_COM_X_fr_world;
    Type_fr_lower_neck_link_COM_X_fr_lower_neck_joint fr_lower_neck_link_COM_X_fr_lower_neck_joint;
    Type_fr_lower_neck_link_COM_X_fr_chassis_joint fr_lower_neck_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_lower_tail_link_COM fr_world_X_fr_lower_tail_link_COM;
    Type_fr_lower_tail_link_COM_X_fr_world fr_lower_tail_link_COM_X_fr_world;
    Type_fr_lower_tail_link_COM_X_fr_lower_tail_joint fr_lower_tail_link_COM_X_fr_lower_tail_joint;
    Type_fr_lower_tail_link_COM_X_fr_upper_tail_joint fr_lower_tail_link_COM_X_fr_upper_tail_joint;
    Type_fr_lower_tail_link_COM_X_fr_chassis_joint fr_lower_tail_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_mouth_link_COM fr_world_X_fr_mouth_link_COM;
    Type_fr_mouth_link_COM_X_fr_world fr_mouth_link_COM_X_fr_world;
    Type_fr_mouth_link_COM_X_fr_mouth_joint fr_mouth_link_COM_X_fr_mouth_joint;
    Type_fr_mouth_link_COM_X_fr_head_joint fr_mouth_link_COM_X_fr_head_joint;
    Type_fr_mouth_link_COM_X_fr_upper_neck_joint fr_mouth_link_COM_X_fr_upper_neck_joint;
    Type_fr_mouth_link_COM_X_fr_lower_neck_joint fr_mouth_link_COM_X_fr_lower_neck_joint;
    Type_fr_mouth_link_COM_X_fr_chassis_joint fr_mouth_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_right_ear_link_COM fr_world_X_fr_right_ear_link_COM;
    Type_fr_right_ear_link_COM_X_fr_world fr_right_ear_link_COM_X_fr_world;
    Type_fr_right_ear_link_COM_X_fr_right_ear_joint fr_right_ear_link_COM_X_fr_right_ear_joint;
    Type_fr_right_ear_link_COM_X_fr_head_joint fr_right_ear_link_COM_X_fr_head_joint;
    Type_fr_right_ear_link_COM_X_fr_upper_neck_joint fr_right_ear_link_COM_X_fr_upper_neck_joint;
    Type_fr_right_ear_link_COM_X_fr_lower_neck_joint fr_right_ear_link_COM_X_fr_lower_neck_joint;
    Type_fr_right_ear_link_COM_X_fr_chassis_joint fr_right_ear_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_right_wheel_link_COM fr_world_X_fr_right_wheel_link_COM;
    Type_fr_right_wheel_link_COM_X_fr_world fr_right_wheel_link_COM_X_fr_world;
    Type_fr_right_wheel_link_COM_X_fr_right_wheel_joint fr_right_wheel_link_COM_X_fr_right_wheel_joint;
    Type_fr_right_wheel_link_COM_X_fr_chassis_joint fr_right_wheel_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_upper_neck_link_COM fr_world_X_fr_upper_neck_link_COM;
    Type_fr_upper_neck_link_COM_X_fr_world fr_upper_neck_link_COM_X_fr_world;
    Type_fr_upper_neck_link_COM_X_fr_upper_neck_joint fr_upper_neck_link_COM_X_fr_upper_neck_joint;
    Type_fr_upper_neck_link_COM_X_fr_lower_neck_joint fr_upper_neck_link_COM_X_fr_lower_neck_joint;
    Type_fr_upper_neck_link_COM_X_fr_chassis_joint fr_upper_neck_link_COM_X_fr_chassis_joint;
    Type_fr_world_X_fr_upper_tail_link_COM fr_world_X_fr_upper_tail_link_COM;
    Type_fr_upper_tail_link_COM_X_fr_world fr_upper_tail_link_COM_X_fr_world;
    Type_fr_upper_tail_link_COM_X_fr_upper_tail_joint fr_upper_tail_link_COM_X_fr_upper_tail_joint;
    Type_fr_upper_tail_link_COM_X_fr_chassis_joint fr_upper_tail_link_COM_X_fr_chassis_joint;
    Type_fr_upper_neck_link_X_fr_lower_neck_link fr_upper_neck_link_X_fr_lower_neck_link;
    Type_fr_lower_neck_link_X_fr_upper_neck_link fr_lower_neck_link_X_fr_upper_neck_link;

protected:

}; //class 'HomogeneousTransforms'

}

using MotionTransforms = tpl::MotionTransforms<rbd::DoubleTrait>;
using ForceTransforms = tpl::ForceTransforms<rbd::DoubleTrait>;
using HomogeneousTransforms = tpl::HomogeneousTransforms<rbd::DoubleTrait>;

}
}

#include "transforms.impl.h"

#endif
