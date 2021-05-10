#ifndef IIT_ROBOGEN__FREA_TRAITS_H_
#define IIT_ROBOGEN__FREA_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace frea {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename frea::JointIdentifiers JointID;
    typedef typename frea::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename frea::tpl::JointState<SCALAR> JointState;



    typedef typename frea::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename frea::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename frea::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename frea::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::frea::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::frea::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::frea::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::frea::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = frea::jointsCount;
    static const int links_count  = frea::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return frea::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return frea::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
