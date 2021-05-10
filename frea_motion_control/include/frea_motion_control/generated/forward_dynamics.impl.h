
// Initialization of static-const data
template <typename TRAIT>
const typename iit::frea::dyn::tpl::ForwardDynamics<TRAIT>::ExtForces
    iit::frea::dyn::tpl::ForwardDynamics<TRAIT>::zeroExtForces(Force::Zero());

template <typename TRAIT>
iit::frea::dyn::tpl::ForwardDynamics<TRAIT>::ForwardDynamics(iit::frea::dyn::tpl::InertiaProperties<TRAIT>& inertia, MTransforms& transforms) :
    inertiaProps( & inertia ),
    motionTransforms( & transforms )
{
    chassis_link_v.setZero();
    chassis_link_c.setZero();
    left_wheel_link_v.setZero();
    left_wheel_link_c.setZero();
    right_wheel_link_v.setZero();
    right_wheel_link_c.setZero();
    lower_neck_link_v.setZero();
    lower_neck_link_c.setZero();
    upper_neck_link_v.setZero();
    upper_neck_link_c.setZero();
    head_link_v.setZero();
    head_link_c.setZero();
    mouth_link_v.setZero();
    mouth_link_c.setZero();
    left_ear_link_v.setZero();
    left_ear_link_c.setZero();
    right_ear_link_v.setZero();
    right_ear_link_c.setZero();
    upper_tail_link_v.setZero();
    upper_tail_link_c.setZero();
    lower_tail_link_v.setZero();
    lower_tail_link_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

template <typename TRAIT>
void iit::frea::dyn::tpl::ForwardDynamics<TRAIT>::fd(
    JointState& qdd,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    chassis_link_AI = inertiaProps->getTensor_chassis_link();
    chassis_link_p = - fext[CHASSIS_LINK];
    left_wheel_link_AI = inertiaProps->getTensor_left_wheel_link();
    left_wheel_link_p = - fext[LEFT_WHEEL_LINK];
    right_wheel_link_AI = inertiaProps->getTensor_right_wheel_link();
    right_wheel_link_p = - fext[RIGHT_WHEEL_LINK];
    lower_neck_link_AI = inertiaProps->getTensor_lower_neck_link();
    lower_neck_link_p = - fext[LOWER_NECK_LINK];
    upper_neck_link_AI = inertiaProps->getTensor_upper_neck_link();
    upper_neck_link_p = - fext[UPPER_NECK_LINK];
    head_link_AI = inertiaProps->getTensor_head_link();
    head_link_p = - fext[HEAD_LINK];
    mouth_link_AI = inertiaProps->getTensor_mouth_link();
    mouth_link_p = - fext[MOUTH_LINK];
    left_ear_link_AI = inertiaProps->getTensor_left_ear_link();
    left_ear_link_p = - fext[LEFT_EAR_LINK];
    right_ear_link_AI = inertiaProps->getTensor_right_ear_link();
    right_ear_link_p = - fext[RIGHT_EAR_LINK];
    upper_tail_link_AI = inertiaProps->getTensor_upper_tail_link();
    upper_tail_link_p = - fext[UPPER_TAIL_LINK];
    lower_tail_link_AI = inertiaProps->getTensor_lower_tail_link();
    lower_tail_link_p = - fext[LOWER_TAIL_LINK];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link chassis_link
    //  - The spatial velocity:
    chassis_link_v(iit::rbd::AZ) = qd(CHASSIS_JOINT);
    
    //  - The bias force term:
    chassis_link_p += iit::rbd::vxIv(qd(CHASSIS_JOINT), chassis_link_AI);
    
    // + Link left_wheel_link
    //  - The spatial velocity:
    left_wheel_link_v = (motionTransforms-> fr_left_wheel_link_X_fr_chassis_link) * chassis_link_v;
    left_wheel_link_v(iit::rbd::AZ) += qd(LEFT_WHEEL_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(left_wheel_link_v, vcross);
    left_wheel_link_c = vcross.col(iit::rbd::AZ) * qd(LEFT_WHEEL_JOINT);
    
    //  - The bias force term:
    left_wheel_link_p += iit::rbd::vxIv(left_wheel_link_v, left_wheel_link_AI);
    
    // + Link right_wheel_link
    //  - The spatial velocity:
    right_wheel_link_v = (motionTransforms-> fr_right_wheel_link_X_fr_chassis_link) * chassis_link_v;
    right_wheel_link_v(iit::rbd::AZ) += qd(RIGHT_WHEEL_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(right_wheel_link_v, vcross);
    right_wheel_link_c = vcross.col(iit::rbd::AZ) * qd(RIGHT_WHEEL_JOINT);
    
    //  - The bias force term:
    right_wheel_link_p += iit::rbd::vxIv(right_wheel_link_v, right_wheel_link_AI);
    
    // + Link lower_neck_link
    //  - The spatial velocity:
    lower_neck_link_v = (motionTransforms-> fr_lower_neck_link_X_fr_chassis_link) * chassis_link_v;
    lower_neck_link_v(iit::rbd::AZ) += qd(LOWER_NECK_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(lower_neck_link_v, vcross);
    lower_neck_link_c = vcross.col(iit::rbd::AZ) * qd(LOWER_NECK_JOINT);
    
    //  - The bias force term:
    lower_neck_link_p += iit::rbd::vxIv(lower_neck_link_v, lower_neck_link_AI);
    
    // + Link upper_neck_link
    //  - The spatial velocity:
    upper_neck_link_v = (motionTransforms-> fr_upper_neck_link_X_fr_lower_neck_link) * lower_neck_link_v;
    upper_neck_link_v(iit::rbd::AZ) += qd(UPPER_NECK_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(upper_neck_link_v, vcross);
    upper_neck_link_c = vcross.col(iit::rbd::AZ) * qd(UPPER_NECK_JOINT);
    
    //  - The bias force term:
    upper_neck_link_p += iit::rbd::vxIv(upper_neck_link_v, upper_neck_link_AI);
    
    // + Link head_link
    //  - The spatial velocity:
    head_link_v = (motionTransforms-> fr_head_link_X_fr_upper_neck_link) * upper_neck_link_v;
    head_link_v(iit::rbd::AZ) += qd(HEAD_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(head_link_v, vcross);
    head_link_c = vcross.col(iit::rbd::AZ) * qd(HEAD_JOINT);
    
    //  - The bias force term:
    head_link_p += iit::rbd::vxIv(head_link_v, head_link_AI);
    
    // + Link mouth_link
    //  - The spatial velocity:
    mouth_link_v = (motionTransforms-> fr_mouth_link_X_fr_head_link) * head_link_v;
    mouth_link_v(iit::rbd::AZ) += qd(MOUTH_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(mouth_link_v, vcross);
    mouth_link_c = vcross.col(iit::rbd::AZ) * qd(MOUTH_JOINT);
    
    //  - The bias force term:
    mouth_link_p += iit::rbd::vxIv(mouth_link_v, mouth_link_AI);
    
    // + Link left_ear_link
    //  - The spatial velocity:
    left_ear_link_v = (motionTransforms-> fr_left_ear_link_X_fr_head_link) * head_link_v;
    left_ear_link_v(iit::rbd::AZ) += qd(LEFT_EAR_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(left_ear_link_v, vcross);
    left_ear_link_c = vcross.col(iit::rbd::AZ) * qd(LEFT_EAR_JOINT);
    
    //  - The bias force term:
    left_ear_link_p += iit::rbd::vxIv(left_ear_link_v, left_ear_link_AI);
    
    // + Link right_ear_link
    //  - The spatial velocity:
    right_ear_link_v = (motionTransforms-> fr_right_ear_link_X_fr_head_link) * head_link_v;
    right_ear_link_v(iit::rbd::AZ) += qd(RIGHT_EAR_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(right_ear_link_v, vcross);
    right_ear_link_c = vcross.col(iit::rbd::AZ) * qd(RIGHT_EAR_JOINT);
    
    //  - The bias force term:
    right_ear_link_p += iit::rbd::vxIv(right_ear_link_v, right_ear_link_AI);
    
    // + Link upper_tail_link
    //  - The spatial velocity:
    upper_tail_link_v = (motionTransforms-> fr_upper_tail_link_X_fr_chassis_link) * chassis_link_v;
    upper_tail_link_v(iit::rbd::AZ) += qd(UPPER_TAIL_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(upper_tail_link_v, vcross);
    upper_tail_link_c = vcross.col(iit::rbd::AZ) * qd(UPPER_TAIL_JOINT);
    
    //  - The bias force term:
    upper_tail_link_p += iit::rbd::vxIv(upper_tail_link_v, upper_tail_link_AI);
    
    // + Link lower_tail_link
    //  - The spatial velocity:
    lower_tail_link_v = (motionTransforms-> fr_lower_tail_link_X_fr_upper_tail_link) * upper_tail_link_v;
    lower_tail_link_v(iit::rbd::AZ) += qd(LOWER_TAIL_JOINT);
    
    //  - The velocity-product acceleration term:
    iit::rbd::motionCrossProductMx<Scalar>(lower_tail_link_v, vcross);
    lower_tail_link_c = vcross.col(iit::rbd::AZ) * qd(LOWER_TAIL_JOINT);
    
    //  - The bias force term:
    lower_tail_link_p += iit::rbd::vxIv(lower_tail_link_v, lower_tail_link_AI);
    
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66S IaB;
    Force pa;
    
    // + Link lower_tail_link
    lower_tail_link_u = tau(LOWER_TAIL_JOINT) - lower_tail_link_p(iit::rbd::AZ);
    lower_tail_link_U = lower_tail_link_AI.col(iit::rbd::AZ);
    lower_tail_link_D = lower_tail_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(lower_tail_link_AI, lower_tail_link_U, lower_tail_link_D, Ia_r);  // same as: Ia_r = lower_tail_link_AI - lower_tail_link_U/lower_tail_link_D * lower_tail_link_U.transpose();
    pa = lower_tail_link_p + Ia_r * lower_tail_link_c + lower_tail_link_U * lower_tail_link_u/lower_tail_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_lower_tail_link_X_fr_upper_tail_link, IaB);
    upper_tail_link_AI += IaB;
    upper_tail_link_p += (motionTransforms-> fr_lower_tail_link_X_fr_upper_tail_link).transpose() * pa;
    
    // + Link upper_tail_link
    upper_tail_link_u = tau(UPPER_TAIL_JOINT) - upper_tail_link_p(iit::rbd::AZ);
    upper_tail_link_U = upper_tail_link_AI.col(iit::rbd::AZ);
    upper_tail_link_D = upper_tail_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(upper_tail_link_AI, upper_tail_link_U, upper_tail_link_D, Ia_r);  // same as: Ia_r = upper_tail_link_AI - upper_tail_link_U/upper_tail_link_D * upper_tail_link_U.transpose();
    pa = upper_tail_link_p + Ia_r * upper_tail_link_c + upper_tail_link_U * upper_tail_link_u/upper_tail_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_upper_tail_link_X_fr_chassis_link, IaB);
    chassis_link_AI += IaB;
    chassis_link_p += (motionTransforms-> fr_upper_tail_link_X_fr_chassis_link).transpose() * pa;
    
    // + Link right_ear_link
    right_ear_link_u = tau(RIGHT_EAR_JOINT) - right_ear_link_p(iit::rbd::AZ);
    right_ear_link_U = right_ear_link_AI.col(iit::rbd::AZ);
    right_ear_link_D = right_ear_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(right_ear_link_AI, right_ear_link_U, right_ear_link_D, Ia_r);  // same as: Ia_r = right_ear_link_AI - right_ear_link_U/right_ear_link_D * right_ear_link_U.transpose();
    pa = right_ear_link_p + Ia_r * right_ear_link_c + right_ear_link_U * right_ear_link_u/right_ear_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_right_ear_link_X_fr_head_link, IaB);
    head_link_AI += IaB;
    head_link_p += (motionTransforms-> fr_right_ear_link_X_fr_head_link).transpose() * pa;
    
    // + Link left_ear_link
    left_ear_link_u = tau(LEFT_EAR_JOINT) - left_ear_link_p(iit::rbd::AZ);
    left_ear_link_U = left_ear_link_AI.col(iit::rbd::AZ);
    left_ear_link_D = left_ear_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(left_ear_link_AI, left_ear_link_U, left_ear_link_D, Ia_r);  // same as: Ia_r = left_ear_link_AI - left_ear_link_U/left_ear_link_D * left_ear_link_U.transpose();
    pa = left_ear_link_p + Ia_r * left_ear_link_c + left_ear_link_U * left_ear_link_u/left_ear_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_left_ear_link_X_fr_head_link, IaB);
    head_link_AI += IaB;
    head_link_p += (motionTransforms-> fr_left_ear_link_X_fr_head_link).transpose() * pa;
    
    // + Link mouth_link
    mouth_link_u = tau(MOUTH_JOINT) - mouth_link_p(iit::rbd::AZ);
    mouth_link_U = mouth_link_AI.col(iit::rbd::AZ);
    mouth_link_D = mouth_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(mouth_link_AI, mouth_link_U, mouth_link_D, Ia_r);  // same as: Ia_r = mouth_link_AI - mouth_link_U/mouth_link_D * mouth_link_U.transpose();
    pa = mouth_link_p + Ia_r * mouth_link_c + mouth_link_U * mouth_link_u/mouth_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_mouth_link_X_fr_head_link, IaB);
    head_link_AI += IaB;
    head_link_p += (motionTransforms-> fr_mouth_link_X_fr_head_link).transpose() * pa;
    
    // + Link head_link
    head_link_u = tau(HEAD_JOINT) - head_link_p(iit::rbd::AZ);
    head_link_U = head_link_AI.col(iit::rbd::AZ);
    head_link_D = head_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(head_link_AI, head_link_U, head_link_D, Ia_r);  // same as: Ia_r = head_link_AI - head_link_U/head_link_D * head_link_U.transpose();
    pa = head_link_p + Ia_r * head_link_c + head_link_U * head_link_u/head_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_head_link_X_fr_upper_neck_link, IaB);
    upper_neck_link_AI += IaB;
    upper_neck_link_p += (motionTransforms-> fr_head_link_X_fr_upper_neck_link).transpose() * pa;
    
    // + Link upper_neck_link
    upper_neck_link_u = tau(UPPER_NECK_JOINT) - upper_neck_link_p(iit::rbd::AZ);
    upper_neck_link_U = upper_neck_link_AI.col(iit::rbd::AZ);
    upper_neck_link_D = upper_neck_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(upper_neck_link_AI, upper_neck_link_U, upper_neck_link_D, Ia_r);  // same as: Ia_r = upper_neck_link_AI - upper_neck_link_U/upper_neck_link_D * upper_neck_link_U.transpose();
    pa = upper_neck_link_p + Ia_r * upper_neck_link_c + upper_neck_link_U * upper_neck_link_u/upper_neck_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_upper_neck_link_X_fr_lower_neck_link, IaB);
    lower_neck_link_AI += IaB;
    lower_neck_link_p += (motionTransforms-> fr_upper_neck_link_X_fr_lower_neck_link).transpose() * pa;
    
    // + Link lower_neck_link
    lower_neck_link_u = tau(LOWER_NECK_JOINT) - lower_neck_link_p(iit::rbd::AZ);
    lower_neck_link_U = lower_neck_link_AI.col(iit::rbd::AZ);
    lower_neck_link_D = lower_neck_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(lower_neck_link_AI, lower_neck_link_U, lower_neck_link_D, Ia_r);  // same as: Ia_r = lower_neck_link_AI - lower_neck_link_U/lower_neck_link_D * lower_neck_link_U.transpose();
    pa = lower_neck_link_p + Ia_r * lower_neck_link_c + lower_neck_link_U * lower_neck_link_u/lower_neck_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_lower_neck_link_X_fr_chassis_link, IaB);
    chassis_link_AI += IaB;
    chassis_link_p += (motionTransforms-> fr_lower_neck_link_X_fr_chassis_link).transpose() * pa;
    
    // + Link right_wheel_link
    right_wheel_link_u = tau(RIGHT_WHEEL_JOINT) - right_wheel_link_p(iit::rbd::AZ);
    right_wheel_link_U = right_wheel_link_AI.col(iit::rbd::AZ);
    right_wheel_link_D = right_wheel_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(right_wheel_link_AI, right_wheel_link_U, right_wheel_link_D, Ia_r);  // same as: Ia_r = right_wheel_link_AI - right_wheel_link_U/right_wheel_link_D * right_wheel_link_U.transpose();
    pa = right_wheel_link_p + Ia_r * right_wheel_link_c + right_wheel_link_U * right_wheel_link_u/right_wheel_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_right_wheel_link_X_fr_chassis_link, IaB);
    chassis_link_AI += IaB;
    chassis_link_p += (motionTransforms-> fr_right_wheel_link_X_fr_chassis_link).transpose() * pa;
    
    // + Link left_wheel_link
    left_wheel_link_u = tau(LEFT_WHEEL_JOINT) - left_wheel_link_p(iit::rbd::AZ);
    left_wheel_link_U = left_wheel_link_AI.col(iit::rbd::AZ);
    left_wheel_link_D = left_wheel_link_U(iit::rbd::AZ);
    
    iit::rbd::compute_Ia_revolute(left_wheel_link_AI, left_wheel_link_U, left_wheel_link_D, Ia_r);  // same as: Ia_r = left_wheel_link_AI - left_wheel_link_U/left_wheel_link_D * left_wheel_link_U.transpose();
    pa = left_wheel_link_p + Ia_r * left_wheel_link_c + left_wheel_link_U * left_wheel_link_u/left_wheel_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_left_wheel_link_X_fr_chassis_link, IaB);
    chassis_link_AI += IaB;
    chassis_link_p += (motionTransforms-> fr_left_wheel_link_X_fr_chassis_link).transpose() * pa;
    
    // + Link chassis_link
    chassis_link_u = tau(CHASSIS_JOINT) - chassis_link_p(iit::rbd::AZ);
    chassis_link_U = chassis_link_AI.col(iit::rbd::AZ);
    chassis_link_D = chassis_link_U(iit::rbd::AZ);
    
    
    
    // ---------------------- THIRD PASS ---------------------- //
    chassis_link_a = (motionTransforms-> fr_chassis_link_X_fr_world).col(iit::rbd::LZ) * Scalar(iit::rbd::g);
    qdd(CHASSIS_JOINT) = (chassis_link_u - chassis_link_U.dot(chassis_link_a)) / chassis_link_D;
    chassis_link_a(iit::rbd::AZ) += qdd(CHASSIS_JOINT);
    
    left_wheel_link_a = (motionTransforms-> fr_left_wheel_link_X_fr_chassis_link) * chassis_link_a + left_wheel_link_c;
    qdd(LEFT_WHEEL_JOINT) = (left_wheel_link_u - left_wheel_link_U.dot(left_wheel_link_a)) / left_wheel_link_D;
    left_wheel_link_a(iit::rbd::AZ) += qdd(LEFT_WHEEL_JOINT);
    
    right_wheel_link_a = (motionTransforms-> fr_right_wheel_link_X_fr_chassis_link) * chassis_link_a + right_wheel_link_c;
    qdd(RIGHT_WHEEL_JOINT) = (right_wheel_link_u - right_wheel_link_U.dot(right_wheel_link_a)) / right_wheel_link_D;
    right_wheel_link_a(iit::rbd::AZ) += qdd(RIGHT_WHEEL_JOINT);
    
    lower_neck_link_a = (motionTransforms-> fr_lower_neck_link_X_fr_chassis_link) * chassis_link_a + lower_neck_link_c;
    qdd(LOWER_NECK_JOINT) = (lower_neck_link_u - lower_neck_link_U.dot(lower_neck_link_a)) / lower_neck_link_D;
    lower_neck_link_a(iit::rbd::AZ) += qdd(LOWER_NECK_JOINT);
    
    upper_neck_link_a = (motionTransforms-> fr_upper_neck_link_X_fr_lower_neck_link) * lower_neck_link_a + upper_neck_link_c;
    qdd(UPPER_NECK_JOINT) = (upper_neck_link_u - upper_neck_link_U.dot(upper_neck_link_a)) / upper_neck_link_D;
    upper_neck_link_a(iit::rbd::AZ) += qdd(UPPER_NECK_JOINT);
    
    head_link_a = (motionTransforms-> fr_head_link_X_fr_upper_neck_link) * upper_neck_link_a + head_link_c;
    qdd(HEAD_JOINT) = (head_link_u - head_link_U.dot(head_link_a)) / head_link_D;
    head_link_a(iit::rbd::AZ) += qdd(HEAD_JOINT);
    
    mouth_link_a = (motionTransforms-> fr_mouth_link_X_fr_head_link) * head_link_a + mouth_link_c;
    qdd(MOUTH_JOINT) = (mouth_link_u - mouth_link_U.dot(mouth_link_a)) / mouth_link_D;
    mouth_link_a(iit::rbd::AZ) += qdd(MOUTH_JOINT);
    
    left_ear_link_a = (motionTransforms-> fr_left_ear_link_X_fr_head_link) * head_link_a + left_ear_link_c;
    qdd(LEFT_EAR_JOINT) = (left_ear_link_u - left_ear_link_U.dot(left_ear_link_a)) / left_ear_link_D;
    left_ear_link_a(iit::rbd::AZ) += qdd(LEFT_EAR_JOINT);
    
    right_ear_link_a = (motionTransforms-> fr_right_ear_link_X_fr_head_link) * head_link_a + right_ear_link_c;
    qdd(RIGHT_EAR_JOINT) = (right_ear_link_u - right_ear_link_U.dot(right_ear_link_a)) / right_ear_link_D;
    right_ear_link_a(iit::rbd::AZ) += qdd(RIGHT_EAR_JOINT);
    
    upper_tail_link_a = (motionTransforms-> fr_upper_tail_link_X_fr_chassis_link) * chassis_link_a + upper_tail_link_c;
    qdd(UPPER_TAIL_JOINT) = (upper_tail_link_u - upper_tail_link_U.dot(upper_tail_link_a)) / upper_tail_link_D;
    upper_tail_link_a(iit::rbd::AZ) += qdd(UPPER_TAIL_JOINT);
    
    lower_tail_link_a = (motionTransforms-> fr_lower_tail_link_X_fr_upper_tail_link) * upper_tail_link_a + lower_tail_link_c;
    qdd(LOWER_TAIL_JOINT) = (lower_tail_link_u - lower_tail_link_U.dot(lower_tail_link_a)) / lower_tail_link_D;
    lower_tail_link_a(iit::rbd::AZ) += qdd(LOWER_TAIL_JOINT);
    
    
}
