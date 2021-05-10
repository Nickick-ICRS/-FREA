#ifndef IIT_FREA_LINK_DATA_MAP_H_
#define IIT_FREA_LINK_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace frea {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[WORLD] = rhs[WORLD];
    data[CHASSIS_LINK] = rhs[CHASSIS_LINK];
    data[LEFT_WHEEL_LINK] = rhs[LEFT_WHEEL_LINK];
    data[RIGHT_WHEEL_LINK] = rhs[RIGHT_WHEEL_LINK];
    data[LOWER_NECK_LINK] = rhs[LOWER_NECK_LINK];
    data[UPPER_NECK_LINK] = rhs[UPPER_NECK_LINK];
    data[HEAD_LINK] = rhs[HEAD_LINK];
    data[MOUTH_LINK] = rhs[MOUTH_LINK];
    data[LEFT_EAR_LINK] = rhs[LEFT_EAR_LINK];
    data[RIGHT_EAR_LINK] = rhs[RIGHT_EAR_LINK];
    data[UPPER_TAIL_LINK] = rhs[UPPER_TAIL_LINK];
    data[LOWER_TAIL_LINK] = rhs[LOWER_TAIL_LINK];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[WORLD] = value;
    data[CHASSIS_LINK] = value;
    data[LEFT_WHEEL_LINK] = value;
    data[RIGHT_WHEEL_LINK] = value;
    data[LOWER_NECK_LINK] = value;
    data[UPPER_NECK_LINK] = value;
    data[HEAD_LINK] = value;
    data[MOUTH_LINK] = value;
    data[LEFT_EAR_LINK] = value;
    data[RIGHT_EAR_LINK] = value;
    data[UPPER_TAIL_LINK] = value;
    data[LOWER_TAIL_LINK] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   world = "
    << map[WORLD]
    << "   chassis_link = "
    << map[CHASSIS_LINK]
    << "   left_wheel_link = "
    << map[LEFT_WHEEL_LINK]
    << "   right_wheel_link = "
    << map[RIGHT_WHEEL_LINK]
    << "   lower_neck_link = "
    << map[LOWER_NECK_LINK]
    << "   upper_neck_link = "
    << map[UPPER_NECK_LINK]
    << "   head_link = "
    << map[HEAD_LINK]
    << "   mouth_link = "
    << map[MOUTH_LINK]
    << "   left_ear_link = "
    << map[LEFT_EAR_LINK]
    << "   right_ear_link = "
    << map[RIGHT_EAR_LINK]
    << "   upper_tail_link = "
    << map[UPPER_TAIL_LINK]
    << "   lower_tail_link = "
    << map[LOWER_TAIL_LINK]
    ;
    return out;
}

}
}
#endif
