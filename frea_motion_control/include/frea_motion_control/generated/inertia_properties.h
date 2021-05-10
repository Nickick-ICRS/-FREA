#ifndef IIT_ROBOT_FREA_INERTIA_PROPERTIES_H_
#define IIT_ROBOT_FREA_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/traits/DoubleTrait.h>

#include "declarations.h"

namespace iit {
namespace frea {
/**
 * This namespace encloses classes and functions related to the Dynamics
 * of the robot frea.
 */
namespace dyn {

using InertiaMatrix = iit::rbd::InertiaMatrixDense;

namespace tpl {

template <typename TRAIT>
class InertiaProperties {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar Scalar;
        typedef iit::rbd::Core<Scalar> CoreS;
        typedef iit::rbd::tpl::InertiaMatrixDense<Scalar> IMatrix;
        typedef typename CoreS::Vector3 Vec3d;

        InertiaProperties();
        ~InertiaProperties();
        const IMatrix& getTensor_chassis_link() const;
        const IMatrix& getTensor_left_wheel_link() const;
        const IMatrix& getTensor_right_wheel_link() const;
        const IMatrix& getTensor_lower_neck_link() const;
        const IMatrix& getTensor_upper_neck_link() const;
        const IMatrix& getTensor_head_link() const;
        const IMatrix& getTensor_mouth_link() const;
        const IMatrix& getTensor_left_ear_link() const;
        const IMatrix& getTensor_right_ear_link() const;
        const IMatrix& getTensor_upper_tail_link() const;
        const IMatrix& getTensor_lower_tail_link() const;
        Scalar getMass_chassis_link() const;
        Scalar getMass_left_wheel_link() const;
        Scalar getMass_right_wheel_link() const;
        Scalar getMass_lower_neck_link() const;
        Scalar getMass_upper_neck_link() const;
        Scalar getMass_head_link() const;
        Scalar getMass_mouth_link() const;
        Scalar getMass_left_ear_link() const;
        Scalar getMass_right_ear_link() const;
        Scalar getMass_upper_tail_link() const;
        Scalar getMass_lower_tail_link() const;
        const Vec3d& getCOM_chassis_link() const;
        const Vec3d& getCOM_left_wheel_link() const;
        const Vec3d& getCOM_right_wheel_link() const;
        const Vec3d& getCOM_lower_neck_link() const;
        const Vec3d& getCOM_upper_neck_link() const;
        const Vec3d& getCOM_head_link() const;
        const Vec3d& getCOM_mouth_link() const;
        const Vec3d& getCOM_left_ear_link() const;
        const Vec3d& getCOM_right_ear_link() const;
        const Vec3d& getCOM_upper_tail_link() const;
        const Vec3d& getCOM_lower_tail_link() const;
        Scalar getTotalMass() const;

    private:

        IMatrix tensor_chassis_link;
        IMatrix tensor_left_wheel_link;
        IMatrix tensor_right_wheel_link;
        IMatrix tensor_lower_neck_link;
        IMatrix tensor_upper_neck_link;
        IMatrix tensor_head_link;
        IMatrix tensor_mouth_link;
        IMatrix tensor_left_ear_link;
        IMatrix tensor_right_ear_link;
        IMatrix tensor_upper_tail_link;
        IMatrix tensor_lower_tail_link;
        Vec3d com_chassis_link;
        Vec3d com_left_wheel_link;
        Vec3d com_right_wheel_link;
        Vec3d com_lower_neck_link;
        Vec3d com_upper_neck_link;
        Vec3d com_head_link;
        Vec3d com_mouth_link;
        Vec3d com_left_ear_link;
        Vec3d com_right_ear_link;
        Vec3d com_upper_tail_link;
        Vec3d com_lower_tail_link;
};

template <typename TRAIT>
inline InertiaProperties<TRAIT>::~InertiaProperties() {}

template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_chassis_link() const {
    return this->tensor_chassis_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_left_wheel_link() const {
    return this->tensor_left_wheel_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_right_wheel_link() const {
    return this->tensor_right_wheel_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_lower_neck_link() const {
    return this->tensor_lower_neck_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_upper_neck_link() const {
    return this->tensor_upper_neck_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_head_link() const {
    return this->tensor_head_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_mouth_link() const {
    return this->tensor_mouth_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_left_ear_link() const {
    return this->tensor_left_ear_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_right_ear_link() const {
    return this->tensor_right_ear_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_upper_tail_link() const {
    return this->tensor_upper_tail_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::IMatrix& InertiaProperties<TRAIT>::getTensor_lower_tail_link() const {
    return this->tensor_lower_tail_link;
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_chassis_link() const {
    return this->tensor_chassis_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_left_wheel_link() const {
    return this->tensor_left_wheel_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_right_wheel_link() const {
    return this->tensor_right_wheel_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_lower_neck_link() const {
    return this->tensor_lower_neck_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_upper_neck_link() const {
    return this->tensor_upper_neck_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_head_link() const {
    return this->tensor_head_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_mouth_link() const {
    return this->tensor_mouth_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_left_ear_link() const {
    return this->tensor_left_ear_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_right_ear_link() const {
    return this->tensor_right_ear_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_upper_tail_link() const {
    return this->tensor_upper_tail_link.getMass();
}
template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getMass_lower_tail_link() const {
    return this->tensor_lower_tail_link.getMass();
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_chassis_link() const {
    return this->com_chassis_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_left_wheel_link() const {
    return this->com_left_wheel_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_right_wheel_link() const {
    return this->com_right_wheel_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_lower_neck_link() const {
    return this->com_lower_neck_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_upper_neck_link() const {
    return this->com_upper_neck_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_head_link() const {
    return this->com_head_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_mouth_link() const {
    return this->com_mouth_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_left_ear_link() const {
    return this->com_left_ear_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_right_ear_link() const {
    return this->com_right_ear_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_upper_tail_link() const {
    return this->com_upper_tail_link;
}
template <typename TRAIT>
inline const typename InertiaProperties<TRAIT>::Vec3d& InertiaProperties<TRAIT>::getCOM_lower_tail_link() const {
    return this->com_lower_tail_link;
}

template <typename TRAIT>
inline typename InertiaProperties<TRAIT>::Scalar InertiaProperties<TRAIT>::getTotalMass() const {
    return 5.955353 + 0.880493 + 0.880493 + 0.919374 + 0.099676 + 0.743802 + 0.034405 + 0.016118 + 0.01533 + 0.919374 + 0.099676;
}

}

using InertiaProperties = tpl::InertiaProperties<rbd::DoubleTrait>;

}
}
}

#include "inertia_properties.impl.h"

#endif
