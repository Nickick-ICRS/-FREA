#ifndef FREA_JACOBIANS_H_
#define FREA_JACOBIANS_H_

		#include <iit/rbd/rbd.h>
#include <iit/rbd/TransformsBase.h>
#include <iit/rbd/traits/DoubleTrait.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace frea {

template<typename SCALAR, int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<tpl::JointState<SCALAR>, COLS, M>
{};

namespace tpl {

/**
 *
 */
template <typename TRAIT>
class Jacobians {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar Scalar;
        typedef iit::rbd::Core<Scalar> CoreS;

        typedef JointState<Scalar> JState;

        class Type_fr_world_J_fr_chassis_link : public JacobianT<Scalar, 1, Type_fr_world_J_fr_chassis_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_chassis_link();
            const Type_fr_world_J_fr_chassis_link& update(const JState&);
        protected:
        };
        
        class Type_fr_chassis_link_J_fr_world : public JacobianT<Scalar, 1, Type_fr_chassis_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_chassis_link_J_fr_world();
            const Type_fr_chassis_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_head_link : public JacobianT<Scalar, 4, Type_fr_world_J_fr_head_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_head_link();
            const Type_fr_world_J_fr_head_link& update(const JState&);
        protected:
        };
        
        class Type_fr_head_link_J_fr_world : public JacobianT<Scalar, 4, Type_fr_head_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_head_link_J_fr_world();
            const Type_fr_head_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_left_ear_link : public JacobianT<Scalar, 5, Type_fr_world_J_fr_left_ear_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_left_ear_link();
            const Type_fr_world_J_fr_left_ear_link& update(const JState&);
        protected:
        };
        
        class Type_fr_left_ear_link_J_fr_world : public JacobianT<Scalar, 5, Type_fr_left_ear_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_left_ear_link_J_fr_world();
            const Type_fr_left_ear_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_left_wheel_link : public JacobianT<Scalar, 2, Type_fr_world_J_fr_left_wheel_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_left_wheel_link();
            const Type_fr_world_J_fr_left_wheel_link& update(const JState&);
        protected:
        };
        
        class Type_fr_left_wheel_link_J_fr_world : public JacobianT<Scalar, 2, Type_fr_left_wheel_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_left_wheel_link_J_fr_world();
            const Type_fr_left_wheel_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_lower_neck_link : public JacobianT<Scalar, 2, Type_fr_world_J_fr_lower_neck_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_lower_neck_link();
            const Type_fr_world_J_fr_lower_neck_link& update(const JState&);
        protected:
        };
        
        class Type_fr_lower_neck_link_J_fr_world : public JacobianT<Scalar, 2, Type_fr_lower_neck_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_lower_neck_link_J_fr_world();
            const Type_fr_lower_neck_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_lower_tail_link : public JacobianT<Scalar, 3, Type_fr_world_J_fr_lower_tail_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_lower_tail_link();
            const Type_fr_world_J_fr_lower_tail_link& update(const JState&);
        protected:
        };
        
        class Type_fr_lower_tail_link_J_fr_world : public JacobianT<Scalar, 3, Type_fr_lower_tail_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_lower_tail_link_J_fr_world();
            const Type_fr_lower_tail_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_mouth_link : public JacobianT<Scalar, 5, Type_fr_world_J_fr_mouth_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_mouth_link();
            const Type_fr_world_J_fr_mouth_link& update(const JState&);
        protected:
        };
        
        class Type_fr_mouth_link_J_fr_world : public JacobianT<Scalar, 5, Type_fr_mouth_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_mouth_link_J_fr_world();
            const Type_fr_mouth_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_right_ear_link : public JacobianT<Scalar, 5, Type_fr_world_J_fr_right_ear_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_right_ear_link();
            const Type_fr_world_J_fr_right_ear_link& update(const JState&);
        protected:
        };
        
        class Type_fr_right_ear_link_J_fr_world : public JacobianT<Scalar, 5, Type_fr_right_ear_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_right_ear_link_J_fr_world();
            const Type_fr_right_ear_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_right_wheel_link : public JacobianT<Scalar, 2, Type_fr_world_J_fr_right_wheel_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_right_wheel_link();
            const Type_fr_world_J_fr_right_wheel_link& update(const JState&);
        protected:
        };
        
        class Type_fr_right_wheel_link_J_fr_world : public JacobianT<Scalar, 2, Type_fr_right_wheel_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_right_wheel_link_J_fr_world();
            const Type_fr_right_wheel_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_upper_neck_link : public JacobianT<Scalar, 3, Type_fr_world_J_fr_upper_neck_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_upper_neck_link();
            const Type_fr_world_J_fr_upper_neck_link& update(const JState&);
        protected:
        };
        
        class Type_fr_upper_neck_link_J_fr_world : public JacobianT<Scalar, 3, Type_fr_upper_neck_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_upper_neck_link_J_fr_world();
            const Type_fr_upper_neck_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_upper_tail_link : public JacobianT<Scalar, 2, Type_fr_world_J_fr_upper_tail_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_upper_tail_link();
            const Type_fr_world_J_fr_upper_tail_link& update(const JState&);
        protected:
        };
        
        class Type_fr_upper_tail_link_J_fr_world : public JacobianT<Scalar, 2, Type_fr_upper_tail_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_upper_tail_link_J_fr_world();
            const Type_fr_upper_tail_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_chassis_link_COM : public JacobianT<Scalar, 1, Type_fr_world_J_fr_chassis_link_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_chassis_link_COM();
            const Type_fr_world_J_fr_chassis_link_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_chassis_link_COM_J_fr_world : public JacobianT<Scalar, 1, Type_fr_chassis_link_COM_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_chassis_link_COM_J_fr_world();
            const Type_fr_chassis_link_COM_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_fixed_base_link : public JacobianT<Scalar, 0, Type_fr_world_J_fr_fixed_base_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_fixed_base_link();
            const Type_fr_world_J_fr_fixed_base_link& update(const JState&);
        protected:
        };
        
        class Type_fr_fixed_base_link_J_fr_world : public JacobianT<Scalar, 0, Type_fr_fixed_base_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_fixed_base_link_J_fr_world();
            const Type_fr_fixed_base_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_head_camera_link : public JacobianT<Scalar, 4, Type_fr_world_J_fr_head_camera_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_head_camera_link();
            const Type_fr_world_J_fr_head_camera_link& update(const JState&);
        protected:
        };
        
        class Type_fr_head_camera_link_J_fr_world : public JacobianT<Scalar, 4, Type_fr_head_camera_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_head_camera_link_J_fr_world();
            const Type_fr_head_camera_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_head_link_COM : public JacobianT<Scalar, 4, Type_fr_world_J_fr_head_link_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_head_link_COM();
            const Type_fr_world_J_fr_head_link_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_head_link_COM_J_fr_world : public JacobianT<Scalar, 4, Type_fr_head_link_COM_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_head_link_COM_J_fr_world();
            const Type_fr_head_link_COM_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_imu_link : public JacobianT<Scalar, 1, Type_fr_world_J_fr_imu_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_imu_link();
            const Type_fr_world_J_fr_imu_link& update(const JState&);
        protected:
        };
        
        class Type_fr_imu_link_J_fr_world : public JacobianT<Scalar, 1, Type_fr_imu_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_imu_link_J_fr_world();
            const Type_fr_imu_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_left_ear_link_COM : public JacobianT<Scalar, 5, Type_fr_world_J_fr_left_ear_link_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_left_ear_link_COM();
            const Type_fr_world_J_fr_left_ear_link_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_left_ear_link_COM_J_fr_world : public JacobianT<Scalar, 5, Type_fr_left_ear_link_COM_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_left_ear_link_COM_J_fr_world();
            const Type_fr_left_ear_link_COM_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_left_wheel_link_COM : public JacobianT<Scalar, 2, Type_fr_world_J_fr_left_wheel_link_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_left_wheel_link_COM();
            const Type_fr_world_J_fr_left_wheel_link_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_left_wheel_link_COM_J_fr_world : public JacobianT<Scalar, 2, Type_fr_left_wheel_link_COM_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_left_wheel_link_COM_J_fr_world();
            const Type_fr_left_wheel_link_COM_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_lower_neck_link_COM : public JacobianT<Scalar, 2, Type_fr_world_J_fr_lower_neck_link_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_lower_neck_link_COM();
            const Type_fr_world_J_fr_lower_neck_link_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_lower_neck_link_COM_J_fr_world : public JacobianT<Scalar, 2, Type_fr_lower_neck_link_COM_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_lower_neck_link_COM_J_fr_world();
            const Type_fr_lower_neck_link_COM_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_lower_tail_link_COM : public JacobianT<Scalar, 3, Type_fr_world_J_fr_lower_tail_link_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_lower_tail_link_COM();
            const Type_fr_world_J_fr_lower_tail_link_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_lower_tail_link_COM_J_fr_world : public JacobianT<Scalar, 3, Type_fr_lower_tail_link_COM_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_lower_tail_link_COM_J_fr_world();
            const Type_fr_lower_tail_link_COM_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_mouth_link_COM : public JacobianT<Scalar, 5, Type_fr_world_J_fr_mouth_link_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_mouth_link_COM();
            const Type_fr_world_J_fr_mouth_link_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_mouth_link_COM_J_fr_world : public JacobianT<Scalar, 5, Type_fr_mouth_link_COM_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_mouth_link_COM_J_fr_world();
            const Type_fr_mouth_link_COM_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_parent_link : public JacobianT<Scalar, 0, Type_fr_world_J_fr_parent_link>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_parent_link();
            const Type_fr_world_J_fr_parent_link& update(const JState&);
        protected:
        };
        
        class Type_fr_parent_link_J_fr_world : public JacobianT<Scalar, 0, Type_fr_parent_link_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_parent_link_J_fr_world();
            const Type_fr_parent_link_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_right_ear_link_COM : public JacobianT<Scalar, 5, Type_fr_world_J_fr_right_ear_link_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_right_ear_link_COM();
            const Type_fr_world_J_fr_right_ear_link_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_right_ear_link_COM_J_fr_world : public JacobianT<Scalar, 5, Type_fr_right_ear_link_COM_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_right_ear_link_COM_J_fr_world();
            const Type_fr_right_ear_link_COM_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_right_wheel_link_COM : public JacobianT<Scalar, 2, Type_fr_world_J_fr_right_wheel_link_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_right_wheel_link_COM();
            const Type_fr_world_J_fr_right_wheel_link_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_right_wheel_link_COM_J_fr_world : public JacobianT<Scalar, 2, Type_fr_right_wheel_link_COM_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_right_wheel_link_COM_J_fr_world();
            const Type_fr_right_wheel_link_COM_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_upper_neck_link_COM : public JacobianT<Scalar, 3, Type_fr_world_J_fr_upper_neck_link_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_upper_neck_link_COM();
            const Type_fr_world_J_fr_upper_neck_link_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_upper_neck_link_COM_J_fr_world : public JacobianT<Scalar, 3, Type_fr_upper_neck_link_COM_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_upper_neck_link_COM_J_fr_world();
            const Type_fr_upper_neck_link_COM_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_upper_tail_link_COM : public JacobianT<Scalar, 2, Type_fr_world_J_fr_upper_tail_link_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_upper_tail_link_COM();
            const Type_fr_world_J_fr_upper_tail_link_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_upper_tail_link_COM_J_fr_world : public JacobianT<Scalar, 2, Type_fr_upper_tail_link_COM_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_upper_tail_link_COM_J_fr_world();
            const Type_fr_upper_tail_link_COM_J_fr_world& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_world_COM : public JacobianT<Scalar, 0, Type_fr_world_J_fr_world_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_world_COM();
            const Type_fr_world_J_fr_world_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_world_COM_J_fr_world : public JacobianT<Scalar, 0, Type_fr_world_COM_J_fr_world>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_COM_J_fr_world();
            const Type_fr_world_COM_J_fr_world& update(const JState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_world_J_fr_chassis_link fr_world_J_fr_chassis_link;
        Type_fr_chassis_link_J_fr_world fr_chassis_link_J_fr_world;
        Type_fr_world_J_fr_head_link fr_world_J_fr_head_link;
        Type_fr_head_link_J_fr_world fr_head_link_J_fr_world;
        Type_fr_world_J_fr_left_ear_link fr_world_J_fr_left_ear_link;
        Type_fr_left_ear_link_J_fr_world fr_left_ear_link_J_fr_world;
        Type_fr_world_J_fr_left_wheel_link fr_world_J_fr_left_wheel_link;
        Type_fr_left_wheel_link_J_fr_world fr_left_wheel_link_J_fr_world;
        Type_fr_world_J_fr_lower_neck_link fr_world_J_fr_lower_neck_link;
        Type_fr_lower_neck_link_J_fr_world fr_lower_neck_link_J_fr_world;
        Type_fr_world_J_fr_lower_tail_link fr_world_J_fr_lower_tail_link;
        Type_fr_lower_tail_link_J_fr_world fr_lower_tail_link_J_fr_world;
        Type_fr_world_J_fr_mouth_link fr_world_J_fr_mouth_link;
        Type_fr_mouth_link_J_fr_world fr_mouth_link_J_fr_world;
        Type_fr_world_J_fr_right_ear_link fr_world_J_fr_right_ear_link;
        Type_fr_right_ear_link_J_fr_world fr_right_ear_link_J_fr_world;
        Type_fr_world_J_fr_right_wheel_link fr_world_J_fr_right_wheel_link;
        Type_fr_right_wheel_link_J_fr_world fr_right_wheel_link_J_fr_world;
        Type_fr_world_J_fr_upper_neck_link fr_world_J_fr_upper_neck_link;
        Type_fr_upper_neck_link_J_fr_world fr_upper_neck_link_J_fr_world;
        Type_fr_world_J_fr_upper_tail_link fr_world_J_fr_upper_tail_link;
        Type_fr_upper_tail_link_J_fr_world fr_upper_tail_link_J_fr_world;
        Type_fr_world_J_fr_chassis_link_COM fr_world_J_fr_chassis_link_COM;
        Type_fr_chassis_link_COM_J_fr_world fr_chassis_link_COM_J_fr_world;
        Type_fr_world_J_fr_fixed_base_link fr_world_J_fr_fixed_base_link;
        Type_fr_fixed_base_link_J_fr_world fr_fixed_base_link_J_fr_world;
        Type_fr_world_J_fr_head_camera_link fr_world_J_fr_head_camera_link;
        Type_fr_head_camera_link_J_fr_world fr_head_camera_link_J_fr_world;
        Type_fr_world_J_fr_head_link_COM fr_world_J_fr_head_link_COM;
        Type_fr_head_link_COM_J_fr_world fr_head_link_COM_J_fr_world;
        Type_fr_world_J_fr_imu_link fr_world_J_fr_imu_link;
        Type_fr_imu_link_J_fr_world fr_imu_link_J_fr_world;
        Type_fr_world_J_fr_left_ear_link_COM fr_world_J_fr_left_ear_link_COM;
        Type_fr_left_ear_link_COM_J_fr_world fr_left_ear_link_COM_J_fr_world;
        Type_fr_world_J_fr_left_wheel_link_COM fr_world_J_fr_left_wheel_link_COM;
        Type_fr_left_wheel_link_COM_J_fr_world fr_left_wheel_link_COM_J_fr_world;
        Type_fr_world_J_fr_lower_neck_link_COM fr_world_J_fr_lower_neck_link_COM;
        Type_fr_lower_neck_link_COM_J_fr_world fr_lower_neck_link_COM_J_fr_world;
        Type_fr_world_J_fr_lower_tail_link_COM fr_world_J_fr_lower_tail_link_COM;
        Type_fr_lower_tail_link_COM_J_fr_world fr_lower_tail_link_COM_J_fr_world;
        Type_fr_world_J_fr_mouth_link_COM fr_world_J_fr_mouth_link_COM;
        Type_fr_mouth_link_COM_J_fr_world fr_mouth_link_COM_J_fr_world;
        Type_fr_world_J_fr_parent_link fr_world_J_fr_parent_link;
        Type_fr_parent_link_J_fr_world fr_parent_link_J_fr_world;
        Type_fr_world_J_fr_right_ear_link_COM fr_world_J_fr_right_ear_link_COM;
        Type_fr_right_ear_link_COM_J_fr_world fr_right_ear_link_COM_J_fr_world;
        Type_fr_world_J_fr_right_wheel_link_COM fr_world_J_fr_right_wheel_link_COM;
        Type_fr_right_wheel_link_COM_J_fr_world fr_right_wheel_link_COM_J_fr_world;
        Type_fr_world_J_fr_upper_neck_link_COM fr_world_J_fr_upper_neck_link_COM;
        Type_fr_upper_neck_link_COM_J_fr_world fr_upper_neck_link_COM_J_fr_world;
        Type_fr_world_J_fr_upper_tail_link_COM fr_world_J_fr_upper_tail_link_COM;
        Type_fr_upper_tail_link_COM_J_fr_world fr_upper_tail_link_COM_J_fr_world;
        Type_fr_world_J_fr_world_COM fr_world_J_fr_world_COM;
        Type_fr_world_COM_J_fr_world fr_world_COM_J_fr_world;

    protected:

};

} //namespace tpl

using Jacobians = tpl::Jacobians<rbd::DoubleTrait>;

#include "jacobians.impl.h"


}
}

#endif
