// Initialization of static-const data
template <typename TRAIT>
const typename iit::frea::dyn::tpl::InverseDynamics<TRAIT>::ExtForces
iit::frea::dyn::tpl::InverseDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::frea::dyn::tpl::InverseDynamics<TRAIT>::InverseDynamics(IProperties& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    chassis_link_I(inertiaProps->getTensor_chassis_link() ),
    left_wheel_link_I(inertiaProps->getTensor_left_wheel_link() ),
    right_wheel_link_I(inertiaProps->getTensor_right_wheel_link() ),
    lower_neck_link_I(inertiaProps->getTensor_lower_neck_link() ),
    upper_neck_link_I(inertiaProps->getTensor_upper_neck_link() ),
    head_link_I(inertiaProps->getTensor_head_link() ),
    mouth_link_I(inertiaProps->getTensor_mouth_link() ),
    left_ear_link_I(inertiaProps->getTensor_left_ear_link() ),
    right_ear_link_I(inertiaProps->getTensor_right_ear_link() ),
    upper_tail_link_I(inertiaProps->getTensor_upper_tail_link() ),
    lower_tail_link_I(inertiaProps->getTensor_lower_tail_link() )
    {
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot frea, InverseDynamics<TRAIT>::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    chassis_link_v.setZero();
    left_wheel_link_v.setZero();
    right_wheel_link_v.setZero();
    lower_neck_link_v.setZero();
    upper_neck_link_v.setZero();
    head_link_v.setZero();
    mouth_link_v.setZero();
    left_ear_link_v.setZero();
    right_ear_link_v.setZero();
    upper_tail_link_v.setZero();
    lower_tail_link_v.setZero();

    vcross.setZero();
}

template <typename TRAIT>
void iit::frea::dyn::tpl::InverseDynamics<TRAIT>::id(
    JointState& jForces,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    firstPass(qd, qdd, fext);
    secondPass(jForces);
}

template <typename TRAIT>
void iit::frea::dyn::tpl::InverseDynamics<TRAIT>::G_terms(JointState& jForces)
{
    // Link 'chassis_link'
    chassis_link_a = (xm->fr_chassis_link_X_fr_world).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    chassis_link_f = chassis_link_I * chassis_link_a;
    // Link 'left_wheel_link'
    left_wheel_link_a = (xm->fr_left_wheel_link_X_fr_chassis_link) * chassis_link_a;
    left_wheel_link_f = left_wheel_link_I * left_wheel_link_a;
    // Link 'right_wheel_link'
    right_wheel_link_a = (xm->fr_right_wheel_link_X_fr_chassis_link) * chassis_link_a;
    right_wheel_link_f = right_wheel_link_I * right_wheel_link_a;
    // Link 'lower_neck_link'
    lower_neck_link_a = (xm->fr_lower_neck_link_X_fr_chassis_link) * chassis_link_a;
    lower_neck_link_f = lower_neck_link_I * lower_neck_link_a;
    // Link 'upper_neck_link'
    upper_neck_link_a = (xm->fr_upper_neck_link_X_fr_lower_neck_link) * lower_neck_link_a;
    upper_neck_link_f = upper_neck_link_I * upper_neck_link_a;
    // Link 'head_link'
    head_link_a = (xm->fr_head_link_X_fr_upper_neck_link) * upper_neck_link_a;
    head_link_f = head_link_I * head_link_a;
    // Link 'mouth_link'
    mouth_link_a = (xm->fr_mouth_link_X_fr_head_link) * head_link_a;
    mouth_link_f = mouth_link_I * mouth_link_a;
    // Link 'left_ear_link'
    left_ear_link_a = (xm->fr_left_ear_link_X_fr_head_link) * head_link_a;
    left_ear_link_f = left_ear_link_I * left_ear_link_a;
    // Link 'right_ear_link'
    right_ear_link_a = (xm->fr_right_ear_link_X_fr_head_link) * head_link_a;
    right_ear_link_f = right_ear_link_I * right_ear_link_a;
    // Link 'upper_tail_link'
    upper_tail_link_a = (xm->fr_upper_tail_link_X_fr_chassis_link) * chassis_link_a;
    upper_tail_link_f = upper_tail_link_I * upper_tail_link_a;
    // Link 'lower_tail_link'
    lower_tail_link_a = (xm->fr_lower_tail_link_X_fr_upper_tail_link) * upper_tail_link_a;
    lower_tail_link_f = lower_tail_link_I * lower_tail_link_a;

    secondPass(jForces);
}

template <typename TRAIT>
void iit::frea::dyn::tpl::InverseDynamics<TRAIT>::C_terms(JointState& jForces, const JointState& qd)
{
    // Link 'chassis_link'
    chassis_link_v(iit::rbd::AZ) = qd(CHASSIS_JOINT);   // chassis_link_v = vJ, for the first link of a fixed base robot
    
    chassis_link_f = iit::rbd::vxIv(qd(CHASSIS_JOINT), chassis_link_I);
    
    // Link 'left_wheel_link'
    left_wheel_link_v = ((xm->fr_left_wheel_link_X_fr_chassis_link) * chassis_link_v);
    left_wheel_link_v(iit::rbd::AZ) += qd(LEFT_WHEEL_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(left_wheel_link_v, vcross);
    
    left_wheel_link_a = (vcross.col(iit::rbd::AZ) * qd(LEFT_WHEEL_JOINT));
    
    left_wheel_link_f = left_wheel_link_I * left_wheel_link_a + iit::rbd::vxIv(left_wheel_link_v, left_wheel_link_I);
    
    // Link 'right_wheel_link'
    right_wheel_link_v = ((xm->fr_right_wheel_link_X_fr_chassis_link) * chassis_link_v);
    right_wheel_link_v(iit::rbd::AZ) += qd(RIGHT_WHEEL_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(right_wheel_link_v, vcross);
    
    right_wheel_link_a = (vcross.col(iit::rbd::AZ) * qd(RIGHT_WHEEL_JOINT));
    
    right_wheel_link_f = right_wheel_link_I * right_wheel_link_a + iit::rbd::vxIv(right_wheel_link_v, right_wheel_link_I);
    
    // Link 'lower_neck_link'
    lower_neck_link_v = ((xm->fr_lower_neck_link_X_fr_chassis_link) * chassis_link_v);
    lower_neck_link_v(iit::rbd::AZ) += qd(LOWER_NECK_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(lower_neck_link_v, vcross);
    
    lower_neck_link_a = (vcross.col(iit::rbd::AZ) * qd(LOWER_NECK_JOINT));
    
    lower_neck_link_f = lower_neck_link_I * lower_neck_link_a + iit::rbd::vxIv(lower_neck_link_v, lower_neck_link_I);
    
    // Link 'upper_neck_link'
    upper_neck_link_v = ((xm->fr_upper_neck_link_X_fr_lower_neck_link) * lower_neck_link_v);
    upper_neck_link_v(iit::rbd::AZ) += qd(UPPER_NECK_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(upper_neck_link_v, vcross);
    
    upper_neck_link_a = (xm->fr_upper_neck_link_X_fr_lower_neck_link) * lower_neck_link_a + vcross.col(iit::rbd::AZ) * qd(UPPER_NECK_JOINT);
    
    upper_neck_link_f = upper_neck_link_I * upper_neck_link_a + iit::rbd::vxIv(upper_neck_link_v, upper_neck_link_I);
    
    // Link 'head_link'
    head_link_v = ((xm->fr_head_link_X_fr_upper_neck_link) * upper_neck_link_v);
    head_link_v(iit::rbd::AZ) += qd(HEAD_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(head_link_v, vcross);
    
    head_link_a = (xm->fr_head_link_X_fr_upper_neck_link) * upper_neck_link_a + vcross.col(iit::rbd::AZ) * qd(HEAD_JOINT);
    
    head_link_f = head_link_I * head_link_a + iit::rbd::vxIv(head_link_v, head_link_I);
    
    // Link 'mouth_link'
    mouth_link_v = ((xm->fr_mouth_link_X_fr_head_link) * head_link_v);
    mouth_link_v(iit::rbd::AZ) += qd(MOUTH_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(mouth_link_v, vcross);
    
    mouth_link_a = (xm->fr_mouth_link_X_fr_head_link) * head_link_a + vcross.col(iit::rbd::AZ) * qd(MOUTH_JOINT);
    
    mouth_link_f = mouth_link_I * mouth_link_a + iit::rbd::vxIv(mouth_link_v, mouth_link_I);
    
    // Link 'left_ear_link'
    left_ear_link_v = ((xm->fr_left_ear_link_X_fr_head_link) * head_link_v);
    left_ear_link_v(iit::rbd::AZ) += qd(LEFT_EAR_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(left_ear_link_v, vcross);
    
    left_ear_link_a = (xm->fr_left_ear_link_X_fr_head_link) * head_link_a + vcross.col(iit::rbd::AZ) * qd(LEFT_EAR_JOINT);
    
    left_ear_link_f = left_ear_link_I * left_ear_link_a + iit::rbd::vxIv(left_ear_link_v, left_ear_link_I);
    
    // Link 'right_ear_link'
    right_ear_link_v = ((xm->fr_right_ear_link_X_fr_head_link) * head_link_v);
    right_ear_link_v(iit::rbd::AZ) += qd(RIGHT_EAR_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(right_ear_link_v, vcross);
    
    right_ear_link_a = (xm->fr_right_ear_link_X_fr_head_link) * head_link_a + vcross.col(iit::rbd::AZ) * qd(RIGHT_EAR_JOINT);
    
    right_ear_link_f = right_ear_link_I * right_ear_link_a + iit::rbd::vxIv(right_ear_link_v, right_ear_link_I);
    
    // Link 'upper_tail_link'
    upper_tail_link_v = ((xm->fr_upper_tail_link_X_fr_chassis_link) * chassis_link_v);
    upper_tail_link_v(iit::rbd::AZ) += qd(UPPER_TAIL_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(upper_tail_link_v, vcross);
    
    upper_tail_link_a = (vcross.col(iit::rbd::AZ) * qd(UPPER_TAIL_JOINT));
    
    upper_tail_link_f = upper_tail_link_I * upper_tail_link_a + iit::rbd::vxIv(upper_tail_link_v, upper_tail_link_I);
    
    // Link 'lower_tail_link'
    lower_tail_link_v = ((xm->fr_lower_tail_link_X_fr_upper_tail_link) * upper_tail_link_v);
    lower_tail_link_v(iit::rbd::AZ) += qd(LOWER_TAIL_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(lower_tail_link_v, vcross);
    
    lower_tail_link_a = (xm->fr_lower_tail_link_X_fr_upper_tail_link) * upper_tail_link_a + vcross.col(iit::rbd::AZ) * qd(LOWER_TAIL_JOINT);
    
    lower_tail_link_f = lower_tail_link_I * lower_tail_link_a + iit::rbd::vxIv(lower_tail_link_v, lower_tail_link_I);
    

    secondPass(jForces);
}

template <typename TRAIT>
void iit::frea::dyn::tpl::InverseDynamics<TRAIT>::firstPass(const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    // First pass, link 'chassis_link'
    chassis_link_a = (xm->fr_chassis_link_X_fr_world).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    chassis_link_a(iit::rbd::AZ) += qdd(CHASSIS_JOINT);
    chassis_link_v(iit::rbd::AZ) = qd(CHASSIS_JOINT);   // chassis_link_v = vJ, for the first link of a fixed base robot
    
    chassis_link_f = chassis_link_I * chassis_link_a + iit::rbd::vxIv(qd(CHASSIS_JOINT), chassis_link_I)  - fext[CHASSIS_LINK];
    
    // First pass, link 'left_wheel_link'
    left_wheel_link_v = ((xm->fr_left_wheel_link_X_fr_chassis_link) * chassis_link_v);
    left_wheel_link_v(iit::rbd::AZ) += qd(LEFT_WHEEL_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(left_wheel_link_v, vcross);
    
    left_wheel_link_a = (xm->fr_left_wheel_link_X_fr_chassis_link) * chassis_link_a + vcross.col(iit::rbd::AZ) * qd(LEFT_WHEEL_JOINT);
    left_wheel_link_a(iit::rbd::AZ) += qdd(LEFT_WHEEL_JOINT);
    
    left_wheel_link_f = left_wheel_link_I * left_wheel_link_a + iit::rbd::vxIv(left_wheel_link_v, left_wheel_link_I) - fext[LEFT_WHEEL_LINK];
    
    // First pass, link 'right_wheel_link'
    right_wheel_link_v = ((xm->fr_right_wheel_link_X_fr_chassis_link) * chassis_link_v);
    right_wheel_link_v(iit::rbd::AZ) += qd(RIGHT_WHEEL_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(right_wheel_link_v, vcross);
    
    right_wheel_link_a = (xm->fr_right_wheel_link_X_fr_chassis_link) * chassis_link_a + vcross.col(iit::rbd::AZ) * qd(RIGHT_WHEEL_JOINT);
    right_wheel_link_a(iit::rbd::AZ) += qdd(RIGHT_WHEEL_JOINT);
    
    right_wheel_link_f = right_wheel_link_I * right_wheel_link_a + iit::rbd::vxIv(right_wheel_link_v, right_wheel_link_I) - fext[RIGHT_WHEEL_LINK];
    
    // First pass, link 'lower_neck_link'
    lower_neck_link_v = ((xm->fr_lower_neck_link_X_fr_chassis_link) * chassis_link_v);
    lower_neck_link_v(iit::rbd::AZ) += qd(LOWER_NECK_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(lower_neck_link_v, vcross);
    
    lower_neck_link_a = (xm->fr_lower_neck_link_X_fr_chassis_link) * chassis_link_a + vcross.col(iit::rbd::AZ) * qd(LOWER_NECK_JOINT);
    lower_neck_link_a(iit::rbd::AZ) += qdd(LOWER_NECK_JOINT);
    
    lower_neck_link_f = lower_neck_link_I * lower_neck_link_a + iit::rbd::vxIv(lower_neck_link_v, lower_neck_link_I) - fext[LOWER_NECK_LINK];
    
    // First pass, link 'upper_neck_link'
    upper_neck_link_v = ((xm->fr_upper_neck_link_X_fr_lower_neck_link) * lower_neck_link_v);
    upper_neck_link_v(iit::rbd::AZ) += qd(UPPER_NECK_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(upper_neck_link_v, vcross);
    
    upper_neck_link_a = (xm->fr_upper_neck_link_X_fr_lower_neck_link) * lower_neck_link_a + vcross.col(iit::rbd::AZ) * qd(UPPER_NECK_JOINT);
    upper_neck_link_a(iit::rbd::AZ) += qdd(UPPER_NECK_JOINT);
    
    upper_neck_link_f = upper_neck_link_I * upper_neck_link_a + iit::rbd::vxIv(upper_neck_link_v, upper_neck_link_I) - fext[UPPER_NECK_LINK];
    
    // First pass, link 'head_link'
    head_link_v = ((xm->fr_head_link_X_fr_upper_neck_link) * upper_neck_link_v);
    head_link_v(iit::rbd::AZ) += qd(HEAD_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(head_link_v, vcross);
    
    head_link_a = (xm->fr_head_link_X_fr_upper_neck_link) * upper_neck_link_a + vcross.col(iit::rbd::AZ) * qd(HEAD_JOINT);
    head_link_a(iit::rbd::AZ) += qdd(HEAD_JOINT);
    
    head_link_f = head_link_I * head_link_a + iit::rbd::vxIv(head_link_v, head_link_I) - fext[HEAD_LINK];
    
    // First pass, link 'mouth_link'
    mouth_link_v = ((xm->fr_mouth_link_X_fr_head_link) * head_link_v);
    mouth_link_v(iit::rbd::AZ) += qd(MOUTH_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(mouth_link_v, vcross);
    
    mouth_link_a = (xm->fr_mouth_link_X_fr_head_link) * head_link_a + vcross.col(iit::rbd::AZ) * qd(MOUTH_JOINT);
    mouth_link_a(iit::rbd::AZ) += qdd(MOUTH_JOINT);
    
    mouth_link_f = mouth_link_I * mouth_link_a + iit::rbd::vxIv(mouth_link_v, mouth_link_I) - fext[MOUTH_LINK];
    
    // First pass, link 'left_ear_link'
    left_ear_link_v = ((xm->fr_left_ear_link_X_fr_head_link) * head_link_v);
    left_ear_link_v(iit::rbd::AZ) += qd(LEFT_EAR_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(left_ear_link_v, vcross);
    
    left_ear_link_a = (xm->fr_left_ear_link_X_fr_head_link) * head_link_a + vcross.col(iit::rbd::AZ) * qd(LEFT_EAR_JOINT);
    left_ear_link_a(iit::rbd::AZ) += qdd(LEFT_EAR_JOINT);
    
    left_ear_link_f = left_ear_link_I * left_ear_link_a + iit::rbd::vxIv(left_ear_link_v, left_ear_link_I) - fext[LEFT_EAR_LINK];
    
    // First pass, link 'right_ear_link'
    right_ear_link_v = ((xm->fr_right_ear_link_X_fr_head_link) * head_link_v);
    right_ear_link_v(iit::rbd::AZ) += qd(RIGHT_EAR_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(right_ear_link_v, vcross);
    
    right_ear_link_a = (xm->fr_right_ear_link_X_fr_head_link) * head_link_a + vcross.col(iit::rbd::AZ) * qd(RIGHT_EAR_JOINT);
    right_ear_link_a(iit::rbd::AZ) += qdd(RIGHT_EAR_JOINT);
    
    right_ear_link_f = right_ear_link_I * right_ear_link_a + iit::rbd::vxIv(right_ear_link_v, right_ear_link_I) - fext[RIGHT_EAR_LINK];
    
    // First pass, link 'upper_tail_link'
    upper_tail_link_v = ((xm->fr_upper_tail_link_X_fr_chassis_link) * chassis_link_v);
    upper_tail_link_v(iit::rbd::AZ) += qd(UPPER_TAIL_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(upper_tail_link_v, vcross);
    
    upper_tail_link_a = (xm->fr_upper_tail_link_X_fr_chassis_link) * chassis_link_a + vcross.col(iit::rbd::AZ) * qd(UPPER_TAIL_JOINT);
    upper_tail_link_a(iit::rbd::AZ) += qdd(UPPER_TAIL_JOINT);
    
    upper_tail_link_f = upper_tail_link_I * upper_tail_link_a + iit::rbd::vxIv(upper_tail_link_v, upper_tail_link_I) - fext[UPPER_TAIL_LINK];
    
    // First pass, link 'lower_tail_link'
    lower_tail_link_v = ((xm->fr_lower_tail_link_X_fr_upper_tail_link) * upper_tail_link_v);
    lower_tail_link_v(iit::rbd::AZ) += qd(LOWER_TAIL_JOINT);
    
    iit::rbd::motionCrossProductMx<Scalar>(lower_tail_link_v, vcross);
    
    lower_tail_link_a = (xm->fr_lower_tail_link_X_fr_upper_tail_link) * upper_tail_link_a + vcross.col(iit::rbd::AZ) * qd(LOWER_TAIL_JOINT);
    lower_tail_link_a(iit::rbd::AZ) += qdd(LOWER_TAIL_JOINT);
    
    lower_tail_link_f = lower_tail_link_I * lower_tail_link_a + iit::rbd::vxIv(lower_tail_link_v, lower_tail_link_I) - fext[LOWER_TAIL_LINK];
    
}

template <typename TRAIT>
void iit::frea::dyn::tpl::InverseDynamics<TRAIT>::secondPass(JointState& jForces)
{
    // Link 'lower_tail_link'
    jForces(LOWER_TAIL_JOINT) = lower_tail_link_f(iit::rbd::AZ);
    upper_tail_link_f += xm->fr_lower_tail_link_X_fr_upper_tail_link.transpose() * lower_tail_link_f;
    // Link 'upper_tail_link'
    jForces(UPPER_TAIL_JOINT) = upper_tail_link_f(iit::rbd::AZ);
    chassis_link_f += xm->fr_upper_tail_link_X_fr_chassis_link.transpose() * upper_tail_link_f;
    // Link 'right_ear_link'
    jForces(RIGHT_EAR_JOINT) = right_ear_link_f(iit::rbd::AZ);
    head_link_f += xm->fr_right_ear_link_X_fr_head_link.transpose() * right_ear_link_f;
    // Link 'left_ear_link'
    jForces(LEFT_EAR_JOINT) = left_ear_link_f(iit::rbd::AZ);
    head_link_f += xm->fr_left_ear_link_X_fr_head_link.transpose() * left_ear_link_f;
    // Link 'mouth_link'
    jForces(MOUTH_JOINT) = mouth_link_f(iit::rbd::AZ);
    head_link_f += xm->fr_mouth_link_X_fr_head_link.transpose() * mouth_link_f;
    // Link 'head_link'
    jForces(HEAD_JOINT) = head_link_f(iit::rbd::AZ);
    upper_neck_link_f += xm->fr_head_link_X_fr_upper_neck_link.transpose() * head_link_f;
    // Link 'upper_neck_link'
    jForces(UPPER_NECK_JOINT) = upper_neck_link_f(iit::rbd::AZ);
    lower_neck_link_f += xm->fr_upper_neck_link_X_fr_lower_neck_link.transpose() * upper_neck_link_f;
    // Link 'lower_neck_link'
    jForces(LOWER_NECK_JOINT) = lower_neck_link_f(iit::rbd::AZ);
    chassis_link_f += xm->fr_lower_neck_link_X_fr_chassis_link.transpose() * lower_neck_link_f;
    // Link 'right_wheel_link'
    jForces(RIGHT_WHEEL_JOINT) = right_wheel_link_f(iit::rbd::AZ);
    chassis_link_f += xm->fr_right_wheel_link_X_fr_chassis_link.transpose() * right_wheel_link_f;
    // Link 'left_wheel_link'
    jForces(LEFT_WHEEL_JOINT) = left_wheel_link_f(iit::rbd::AZ);
    chassis_link_f += xm->fr_left_wheel_link_X_fr_chassis_link.transpose() * left_wheel_link_f;
    // Link 'chassis_link'
    jForces(CHASSIS_JOINT) = chassis_link_f(iit::rbd::AZ);
}
