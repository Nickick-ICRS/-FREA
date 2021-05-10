#ifndef __FREA_HPP__
#define __FREA_HPP__

#include <Eigen/Core>
#include <Eigen/StdVector>

// Generated from rcg and installed via sudo make install...
#include "generated/declarations.h"
#include "generated/jsim.h"
#include "generated/jacobians.h"
#include "generated/traits.h"
#include "generated/forward_dynamics.h"
#include "generated/inertia_properties.h"
#include "generated/inverse_dynamics.h"
#include "generated/transforms.h"
#include "generated/link_data_map.h"

// define namespace and base
#define ROBCOGEN_NS frea
#define TARGET_NS frea

// define the links
#define CT_BASE fr_base_link
#define CT_L0 fr_chassis_link
#define CT_L1 fr_left_wheel_link
#define CT_L2 fr_right_wheel_link
#define CT_L3 fr_upper_tail_link
#define CT_L4 fr_lower_tail_link
#define CT_L5 fr_lower_neck_link
#define CT_L6 fr_upper_neck_link
#define CT_L7 fr_head_link
#define CT_L8 fr_mouth_link
#define CT_L9 fr_left_ear_link
#define CT_L10 fr_right_ear_link

// define single end effector (could also be multiple)
//#define CT_N_EE 1
//#define CT_EE0 fr_ee
//#define CT_EE0_IS_ON_LINK 5
//#define CT_EE0_FIRST_JOINT 0
//#define CT_EE0_LAST_JOINT 5
#define CT_N_EE 0

#include <ct/rbd/robot/robcogen/robcogenHelpers.h>

#endif // __FREA_HPP__
