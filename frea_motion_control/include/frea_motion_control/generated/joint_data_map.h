#ifndef IIT_FREA_JOINT_DATA_MAP_H_
#define IIT_FREA_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace frea {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[CHASSIS_JOINT] = rhs[CHASSIS_JOINT];
    data[LEFT_WHEEL_JOINT] = rhs[LEFT_WHEEL_JOINT];
    data[RIGHT_WHEEL_JOINT] = rhs[RIGHT_WHEEL_JOINT];
    data[LOWER_NECK_JOINT] = rhs[LOWER_NECK_JOINT];
    data[UPPER_NECK_JOINT] = rhs[UPPER_NECK_JOINT];
    data[HEAD_JOINT] = rhs[HEAD_JOINT];
    data[MOUTH_JOINT] = rhs[MOUTH_JOINT];
    data[LEFT_EAR_JOINT] = rhs[LEFT_EAR_JOINT];
    data[RIGHT_EAR_JOINT] = rhs[RIGHT_EAR_JOINT];
    data[UPPER_TAIL_JOINT] = rhs[UPPER_TAIL_JOINT];
    data[LOWER_TAIL_JOINT] = rhs[LOWER_TAIL_JOINT];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[CHASSIS_JOINT] = value;
    data[LEFT_WHEEL_JOINT] = value;
    data[RIGHT_WHEEL_JOINT] = value;
    data[LOWER_NECK_JOINT] = value;
    data[UPPER_NECK_JOINT] = value;
    data[HEAD_JOINT] = value;
    data[MOUTH_JOINT] = value;
    data[LEFT_EAR_JOINT] = value;
    data[RIGHT_EAR_JOINT] = value;
    data[UPPER_TAIL_JOINT] = value;
    data[LOWER_TAIL_JOINT] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   chassis_joint = "
    << map[CHASSIS_JOINT]
    << "   left_wheel_joint = "
    << map[LEFT_WHEEL_JOINT]
    << "   right_wheel_joint = "
    << map[RIGHT_WHEEL_JOINT]
    << "   lower_neck_joint = "
    << map[LOWER_NECK_JOINT]
    << "   upper_neck_joint = "
    << map[UPPER_NECK_JOINT]
    << "   head_joint = "
    << map[HEAD_JOINT]
    << "   mouth_joint = "
    << map[MOUTH_JOINT]
    << "   left_ear_joint = "
    << map[LEFT_EAR_JOINT]
    << "   right_ear_joint = "
    << map[RIGHT_EAR_JOINT]
    << "   upper_tail_joint = "
    << map[UPPER_TAIL_JOINT]
    << "   lower_tail_joint = "
    << map[LOWER_TAIL_JOINT]
    ;
    return out;
}

}
}
#endif
