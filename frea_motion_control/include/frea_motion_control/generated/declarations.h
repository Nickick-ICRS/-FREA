#ifndef IIT_ROBOT_FREA_DECLARATIONS_H_
#define IIT_ROBOT_FREA_DECLARATIONS_H_

#include <iit/rbd/rbd.h>

namespace iit {
namespace frea {

static const int JointSpaceDimension = 11;
static const int jointsCount = 11;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 12;

namespace tpl {
template <typename SCALAR>
using Column11d = iit::rbd::PlainMatrix<SCALAR, 11, 1>;

template <typename SCALAR>
using JointState = Column11d<SCALAR>;
}

using Column11d = tpl::Column11d<double>;
typedef Column11d JointState;

enum JointIdentifiers {
    CHASSIS_JOINT = 0
    , LEFT_WHEEL_JOINT
    , RIGHT_WHEEL_JOINT
    , LOWER_NECK_JOINT
    , UPPER_NECK_JOINT
    , HEAD_JOINT
    , MOUTH_JOINT
    , LEFT_EAR_JOINT
    , RIGHT_EAR_JOINT
    , UPPER_TAIL_JOINT
    , LOWER_TAIL_JOINT
};

enum LinkIdentifiers {
    WORLD = 0
    , CHASSIS_LINK
    , LEFT_WHEEL_LINK
    , RIGHT_WHEEL_LINK
    , LOWER_NECK_LINK
    , UPPER_NECK_LINK
    , HEAD_LINK
    , MOUTH_LINK
    , LEFT_EAR_LINK
    , RIGHT_EAR_LINK
    , UPPER_TAIL_LINK
    , LOWER_TAIL_LINK
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {CHASSIS_JOINT,LEFT_WHEEL_JOINT,RIGHT_WHEEL_JOINT,LOWER_NECK_JOINT,UPPER_NECK_JOINT,HEAD_JOINT,MOUTH_JOINT,LEFT_EAR_JOINT,RIGHT_EAR_JOINT,UPPER_TAIL_JOINT,LOWER_TAIL_JOINT};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {WORLD,CHASSIS_LINK,LEFT_WHEEL_LINK,RIGHT_WHEEL_LINK,LOWER_NECK_LINK,UPPER_NECK_LINK,HEAD_LINK,MOUTH_LINK,LEFT_EAR_LINK,RIGHT_EAR_LINK,UPPER_TAIL_LINK,LOWER_TAIL_LINK};

}
}
#endif
