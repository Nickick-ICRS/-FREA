

//Implementation of default constructor
template <typename TRAIT>
iit::frea::dyn::tpl::JSIM<TRAIT>::JSIM(IProperties& inertiaProperties, FTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    left_wheel_link_Ic(linkInertias.getTensor_left_wheel_link()),
    right_wheel_link_Ic(linkInertias.getTensor_right_wheel_link()),
    mouth_link_Ic(linkInertias.getTensor_mouth_link()),
    left_ear_link_Ic(linkInertias.getTensor_left_ear_link()),
    right_ear_link_Ic(linkInertias.getTensor_right_ear_link()),
    lower_tail_link_Ic(linkInertias.getTensor_lower_tail_link())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA tpl::JSIM<TRAIT>::operator()

template <typename TRAIT>
const typename iit::frea::dyn::tpl::JSIM<TRAIT>& iit::frea::dyn::tpl::JSIM<TRAIT>::update(const JointState& state) {
    ForceVector F;

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_upper_tail_link_X_fr_lower_tail_link(state);
    frcTransf -> fr_chassis_link_X_fr_upper_tail_link(state);
    frcTransf -> fr_head_link_X_fr_right_ear_link(state);
    frcTransf -> fr_head_link_X_fr_left_ear_link(state);
    frcTransf -> fr_head_link_X_fr_mouth_link(state);
    frcTransf -> fr_upper_neck_link_X_fr_head_link(state);
    frcTransf -> fr_lower_neck_link_X_fr_upper_neck_link(state);
    frcTransf -> fr_chassis_link_X_fr_lower_neck_link(state);
    frcTransf -> fr_chassis_link_X_fr_right_wheel_link(state);
    frcTransf -> fr_chassis_link_X_fr_left_wheel_link(state);

    // Initializes the composite inertia tensors
    chassis_link_Ic = linkInertias.getTensor_chassis_link();
    lower_neck_link_Ic = linkInertias.getTensor_lower_neck_link();
    upper_neck_link_Ic = linkInertias.getTensor_upper_neck_link();
    head_link_Ic = linkInertias.getTensor_head_link();
    upper_tail_link_Ic = linkInertias.getTensor_upper_tail_link();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link lower_tail_link:
    iit::rbd::transformInertia<Scalar>(lower_tail_link_Ic, frcTransf -> fr_upper_tail_link_X_fr_lower_tail_link, Ic_spare);
    upper_tail_link_Ic += Ic_spare;

    F = lower_tail_link_Ic.col(iit::rbd::AZ);
    DATA(LOWER_TAIL_JOINT, LOWER_TAIL_JOINT) = F(iit::rbd::AZ);

    F = frcTransf -> fr_upper_tail_link_X_fr_lower_tail_link * F;
    DATA(LOWER_TAIL_JOINT, UPPER_TAIL_JOINT) = F(iit::rbd::AZ);
    DATA(UPPER_TAIL_JOINT, LOWER_TAIL_JOINT) = DATA(LOWER_TAIL_JOINT, UPPER_TAIL_JOINT);
    F = frcTransf -> fr_chassis_link_X_fr_upper_tail_link * F;
    DATA(LOWER_TAIL_JOINT, CHASSIS_JOINT) = F(iit::rbd::AZ);
    DATA(CHASSIS_JOINT, LOWER_TAIL_JOINT) = DATA(LOWER_TAIL_JOINT, CHASSIS_JOINT);

    // Link upper_tail_link:
    iit::rbd::transformInertia<Scalar>(upper_tail_link_Ic, frcTransf -> fr_chassis_link_X_fr_upper_tail_link, Ic_spare);
    chassis_link_Ic += Ic_spare;

    F = upper_tail_link_Ic.col(iit::rbd::AZ);
    DATA(UPPER_TAIL_JOINT, UPPER_TAIL_JOINT) = F(iit::rbd::AZ);

    F = frcTransf -> fr_chassis_link_X_fr_upper_tail_link * F;
    DATA(UPPER_TAIL_JOINT, CHASSIS_JOINT) = F(iit::rbd::AZ);
    DATA(CHASSIS_JOINT, UPPER_TAIL_JOINT) = DATA(UPPER_TAIL_JOINT, CHASSIS_JOINT);

    // Link right_ear_link:
    iit::rbd::transformInertia<Scalar>(right_ear_link_Ic, frcTransf -> fr_head_link_X_fr_right_ear_link, Ic_spare);
    head_link_Ic += Ic_spare;

    F = right_ear_link_Ic.col(iit::rbd::AZ);
    DATA(RIGHT_EAR_JOINT, RIGHT_EAR_JOINT) = F(iit::rbd::AZ);

    F = frcTransf -> fr_head_link_X_fr_right_ear_link * F;
    DATA(RIGHT_EAR_JOINT, HEAD_JOINT) = F(iit::rbd::AZ);
    DATA(HEAD_JOINT, RIGHT_EAR_JOINT) = DATA(RIGHT_EAR_JOINT, HEAD_JOINT);
    F = frcTransf -> fr_upper_neck_link_X_fr_head_link * F;
    DATA(RIGHT_EAR_JOINT, UPPER_NECK_JOINT) = F(iit::rbd::AZ);
    DATA(UPPER_NECK_JOINT, RIGHT_EAR_JOINT) = DATA(RIGHT_EAR_JOINT, UPPER_NECK_JOINT);
    F = frcTransf -> fr_lower_neck_link_X_fr_upper_neck_link * F;
    DATA(RIGHT_EAR_JOINT, LOWER_NECK_JOINT) = F(iit::rbd::AZ);
    DATA(LOWER_NECK_JOINT, RIGHT_EAR_JOINT) = DATA(RIGHT_EAR_JOINT, LOWER_NECK_JOINT);
    F = frcTransf -> fr_chassis_link_X_fr_lower_neck_link * F;
    DATA(RIGHT_EAR_JOINT, CHASSIS_JOINT) = F(iit::rbd::AZ);
    DATA(CHASSIS_JOINT, RIGHT_EAR_JOINT) = DATA(RIGHT_EAR_JOINT, CHASSIS_JOINT);

    // Link left_ear_link:
    iit::rbd::transformInertia<Scalar>(left_ear_link_Ic, frcTransf -> fr_head_link_X_fr_left_ear_link, Ic_spare);
    head_link_Ic += Ic_spare;

    F = left_ear_link_Ic.col(iit::rbd::AZ);
    DATA(LEFT_EAR_JOINT, LEFT_EAR_JOINT) = F(iit::rbd::AZ);

    F = frcTransf -> fr_head_link_X_fr_left_ear_link * F;
    DATA(LEFT_EAR_JOINT, HEAD_JOINT) = F(iit::rbd::AZ);
    DATA(HEAD_JOINT, LEFT_EAR_JOINT) = DATA(LEFT_EAR_JOINT, HEAD_JOINT);
    F = frcTransf -> fr_upper_neck_link_X_fr_head_link * F;
    DATA(LEFT_EAR_JOINT, UPPER_NECK_JOINT) = F(iit::rbd::AZ);
    DATA(UPPER_NECK_JOINT, LEFT_EAR_JOINT) = DATA(LEFT_EAR_JOINT, UPPER_NECK_JOINT);
    F = frcTransf -> fr_lower_neck_link_X_fr_upper_neck_link * F;
    DATA(LEFT_EAR_JOINT, LOWER_NECK_JOINT) = F(iit::rbd::AZ);
    DATA(LOWER_NECK_JOINT, LEFT_EAR_JOINT) = DATA(LEFT_EAR_JOINT, LOWER_NECK_JOINT);
    F = frcTransf -> fr_chassis_link_X_fr_lower_neck_link * F;
    DATA(LEFT_EAR_JOINT, CHASSIS_JOINT) = F(iit::rbd::AZ);
    DATA(CHASSIS_JOINT, LEFT_EAR_JOINT) = DATA(LEFT_EAR_JOINT, CHASSIS_JOINT);

    // Link mouth_link:
    iit::rbd::transformInertia<Scalar>(mouth_link_Ic, frcTransf -> fr_head_link_X_fr_mouth_link, Ic_spare);
    head_link_Ic += Ic_spare;

    F = mouth_link_Ic.col(iit::rbd::AZ);
    DATA(MOUTH_JOINT, MOUTH_JOINT) = F(iit::rbd::AZ);

    F = frcTransf -> fr_head_link_X_fr_mouth_link * F;
    DATA(MOUTH_JOINT, HEAD_JOINT) = F(iit::rbd::AZ);
    DATA(HEAD_JOINT, MOUTH_JOINT) = DATA(MOUTH_JOINT, HEAD_JOINT);
    F = frcTransf -> fr_upper_neck_link_X_fr_head_link * F;
    DATA(MOUTH_JOINT, UPPER_NECK_JOINT) = F(iit::rbd::AZ);
    DATA(UPPER_NECK_JOINT, MOUTH_JOINT) = DATA(MOUTH_JOINT, UPPER_NECK_JOINT);
    F = frcTransf -> fr_lower_neck_link_X_fr_upper_neck_link * F;
    DATA(MOUTH_JOINT, LOWER_NECK_JOINT) = F(iit::rbd::AZ);
    DATA(LOWER_NECK_JOINT, MOUTH_JOINT) = DATA(MOUTH_JOINT, LOWER_NECK_JOINT);
    F = frcTransf -> fr_chassis_link_X_fr_lower_neck_link * F;
    DATA(MOUTH_JOINT, CHASSIS_JOINT) = F(iit::rbd::AZ);
    DATA(CHASSIS_JOINT, MOUTH_JOINT) = DATA(MOUTH_JOINT, CHASSIS_JOINT);

    // Link head_link:
    iit::rbd::transformInertia<Scalar>(head_link_Ic, frcTransf -> fr_upper_neck_link_X_fr_head_link, Ic_spare);
    upper_neck_link_Ic += Ic_spare;

    F = head_link_Ic.col(iit::rbd::AZ);
    DATA(HEAD_JOINT, HEAD_JOINT) = F(iit::rbd::AZ);

    F = frcTransf -> fr_upper_neck_link_X_fr_head_link * F;
    DATA(HEAD_JOINT, UPPER_NECK_JOINT) = F(iit::rbd::AZ);
    DATA(UPPER_NECK_JOINT, HEAD_JOINT) = DATA(HEAD_JOINT, UPPER_NECK_JOINT);
    F = frcTransf -> fr_lower_neck_link_X_fr_upper_neck_link * F;
    DATA(HEAD_JOINT, LOWER_NECK_JOINT) = F(iit::rbd::AZ);
    DATA(LOWER_NECK_JOINT, HEAD_JOINT) = DATA(HEAD_JOINT, LOWER_NECK_JOINT);
    F = frcTransf -> fr_chassis_link_X_fr_lower_neck_link * F;
    DATA(HEAD_JOINT, CHASSIS_JOINT) = F(iit::rbd::AZ);
    DATA(CHASSIS_JOINT, HEAD_JOINT) = DATA(HEAD_JOINT, CHASSIS_JOINT);

    // Link upper_neck_link:
    iit::rbd::transformInertia<Scalar>(upper_neck_link_Ic, frcTransf -> fr_lower_neck_link_X_fr_upper_neck_link, Ic_spare);
    lower_neck_link_Ic += Ic_spare;

    F = upper_neck_link_Ic.col(iit::rbd::AZ);
    DATA(UPPER_NECK_JOINT, UPPER_NECK_JOINT) = F(iit::rbd::AZ);

    F = frcTransf -> fr_lower_neck_link_X_fr_upper_neck_link * F;
    DATA(UPPER_NECK_JOINT, LOWER_NECK_JOINT) = F(iit::rbd::AZ);
    DATA(LOWER_NECK_JOINT, UPPER_NECK_JOINT) = DATA(UPPER_NECK_JOINT, LOWER_NECK_JOINT);
    F = frcTransf -> fr_chassis_link_X_fr_lower_neck_link * F;
    DATA(UPPER_NECK_JOINT, CHASSIS_JOINT) = F(iit::rbd::AZ);
    DATA(CHASSIS_JOINT, UPPER_NECK_JOINT) = DATA(UPPER_NECK_JOINT, CHASSIS_JOINT);

    // Link lower_neck_link:
    iit::rbd::transformInertia<Scalar>(lower_neck_link_Ic, frcTransf -> fr_chassis_link_X_fr_lower_neck_link, Ic_spare);
    chassis_link_Ic += Ic_spare;

    F = lower_neck_link_Ic.col(iit::rbd::AZ);
    DATA(LOWER_NECK_JOINT, LOWER_NECK_JOINT) = F(iit::rbd::AZ);

    F = frcTransf -> fr_chassis_link_X_fr_lower_neck_link * F;
    DATA(LOWER_NECK_JOINT, CHASSIS_JOINT) = F(iit::rbd::AZ);
    DATA(CHASSIS_JOINT, LOWER_NECK_JOINT) = DATA(LOWER_NECK_JOINT, CHASSIS_JOINT);

    // Link right_wheel_link:
    iit::rbd::transformInertia<Scalar>(right_wheel_link_Ic, frcTransf -> fr_chassis_link_X_fr_right_wheel_link, Ic_spare);
    chassis_link_Ic += Ic_spare;

    F = right_wheel_link_Ic.col(iit::rbd::AZ);
    DATA(RIGHT_WHEEL_JOINT, RIGHT_WHEEL_JOINT) = F(iit::rbd::AZ);

    F = frcTransf -> fr_chassis_link_X_fr_right_wheel_link * F;
    DATA(RIGHT_WHEEL_JOINT, CHASSIS_JOINT) = F(iit::rbd::AZ);
    DATA(CHASSIS_JOINT, RIGHT_WHEEL_JOINT) = DATA(RIGHT_WHEEL_JOINT, CHASSIS_JOINT);

    // Link left_wheel_link:
    iit::rbd::transformInertia<Scalar>(left_wheel_link_Ic, frcTransf -> fr_chassis_link_X_fr_left_wheel_link, Ic_spare);
    chassis_link_Ic += Ic_spare;

    F = left_wheel_link_Ic.col(iit::rbd::AZ);
    DATA(LEFT_WHEEL_JOINT, LEFT_WHEEL_JOINT) = F(iit::rbd::AZ);

    F = frcTransf -> fr_chassis_link_X_fr_left_wheel_link * F;
    DATA(LEFT_WHEEL_JOINT, CHASSIS_JOINT) = F(iit::rbd::AZ);
    DATA(CHASSIS_JOINT, LEFT_WHEEL_JOINT) = DATA(LEFT_WHEEL_JOINT, CHASSIS_JOINT);

    // Link chassis_link:

    F = chassis_link_Ic.col(iit::rbd::AZ);
    DATA(CHASSIS_JOINT, CHASSIS_JOINT) = F(iit::rbd::AZ);


    return *this;
}

#undef DATA
#undef F

template <typename TRAIT>
void iit::frea::dyn::tpl::JSIM<TRAIT>::computeL() {
    L = this -> template triangularView<Eigen::Lower>();
    // Joint lower_tail_joint, index 10 :
    L(10, 10) = std::sqrt(L(10, 10));
    L(10, 9) = L(10, 9) / L(10, 10);
    L(10, 0) = L(10, 0) / L(10, 10);
    L(9, 9) = L(9, 9) - L(10, 9) * L(10, 9);
    L(9, 0) = L(9, 0) - L(10, 9) * L(10, 0);
    L(0, 0) = L(0, 0) - L(10, 0) * L(10, 0);
    
    // Joint upper_tail_joint, index 9 :
    L(9, 9) = std::sqrt(L(9, 9));
    L(9, 0) = L(9, 0) / L(9, 9);
    L(0, 0) = L(0, 0) - L(9, 0) * L(9, 0);
    
    // Joint right_ear_joint, index 8 :
    L(8, 8) = std::sqrt(L(8, 8));
    L(8, 5) = L(8, 5) / L(8, 8);
    L(8, 4) = L(8, 4) / L(8, 8);
    L(8, 3) = L(8, 3) / L(8, 8);
    L(8, 0) = L(8, 0) / L(8, 8);
    L(5, 5) = L(5, 5) - L(8, 5) * L(8, 5);
    L(5, 4) = L(5, 4) - L(8, 5) * L(8, 4);
    L(5, 3) = L(5, 3) - L(8, 5) * L(8, 3);
    L(5, 0) = L(5, 0) - L(8, 5) * L(8, 0);
    L(4, 4) = L(4, 4) - L(8, 4) * L(8, 4);
    L(4, 3) = L(4, 3) - L(8, 4) * L(8, 3);
    L(4, 0) = L(4, 0) - L(8, 4) * L(8, 0);
    L(3, 3) = L(3, 3) - L(8, 3) * L(8, 3);
    L(3, 0) = L(3, 0) - L(8, 3) * L(8, 0);
    L(0, 0) = L(0, 0) - L(8, 0) * L(8, 0);
    
    // Joint left_ear_joint, index 7 :
    L(7, 7) = std::sqrt(L(7, 7));
    L(7, 5) = L(7, 5) / L(7, 7);
    L(7, 4) = L(7, 4) / L(7, 7);
    L(7, 3) = L(7, 3) / L(7, 7);
    L(7, 0) = L(7, 0) / L(7, 7);
    L(5, 5) = L(5, 5) - L(7, 5) * L(7, 5);
    L(5, 4) = L(5, 4) - L(7, 5) * L(7, 4);
    L(5, 3) = L(5, 3) - L(7, 5) * L(7, 3);
    L(5, 0) = L(5, 0) - L(7, 5) * L(7, 0);
    L(4, 4) = L(4, 4) - L(7, 4) * L(7, 4);
    L(4, 3) = L(4, 3) - L(7, 4) * L(7, 3);
    L(4, 0) = L(4, 0) - L(7, 4) * L(7, 0);
    L(3, 3) = L(3, 3) - L(7, 3) * L(7, 3);
    L(3, 0) = L(3, 0) - L(7, 3) * L(7, 0);
    L(0, 0) = L(0, 0) - L(7, 0) * L(7, 0);
    
    // Joint mouth_joint, index 6 :
    L(6, 6) = std::sqrt(L(6, 6));
    L(6, 5) = L(6, 5) / L(6, 6);
    L(6, 4) = L(6, 4) / L(6, 6);
    L(6, 3) = L(6, 3) / L(6, 6);
    L(6, 0) = L(6, 0) / L(6, 6);
    L(5, 5) = L(5, 5) - L(6, 5) * L(6, 5);
    L(5, 4) = L(5, 4) - L(6, 5) * L(6, 4);
    L(5, 3) = L(5, 3) - L(6, 5) * L(6, 3);
    L(5, 0) = L(5, 0) - L(6, 5) * L(6, 0);
    L(4, 4) = L(4, 4) - L(6, 4) * L(6, 4);
    L(4, 3) = L(4, 3) - L(6, 4) * L(6, 3);
    L(4, 0) = L(4, 0) - L(6, 4) * L(6, 0);
    L(3, 3) = L(3, 3) - L(6, 3) * L(6, 3);
    L(3, 0) = L(3, 0) - L(6, 3) * L(6, 0);
    L(0, 0) = L(0, 0) - L(6, 0) * L(6, 0);
    
    // Joint head_joint, index 5 :
    L(5, 5) = std::sqrt(L(5, 5));
    L(5, 4) = L(5, 4) / L(5, 5);
    L(5, 3) = L(5, 3) / L(5, 5);
    L(5, 0) = L(5, 0) / L(5, 5);
    L(4, 4) = L(4, 4) - L(5, 4) * L(5, 4);
    L(4, 3) = L(4, 3) - L(5, 4) * L(5, 3);
    L(4, 0) = L(4, 0) - L(5, 4) * L(5, 0);
    L(3, 3) = L(3, 3) - L(5, 3) * L(5, 3);
    L(3, 0) = L(3, 0) - L(5, 3) * L(5, 0);
    L(0, 0) = L(0, 0) - L(5, 0) * L(5, 0);
    
    // Joint upper_neck_joint, index 4 :
    L(4, 4) = std::sqrt(L(4, 4));
    L(4, 3) = L(4, 3) / L(4, 4);
    L(4, 0) = L(4, 0) / L(4, 4);
    L(3, 3) = L(3, 3) - L(4, 3) * L(4, 3);
    L(3, 0) = L(3, 0) - L(4, 3) * L(4, 0);
    L(0, 0) = L(0, 0) - L(4, 0) * L(4, 0);
    
    // Joint lower_neck_joint, index 3 :
    L(3, 3) = std::sqrt(L(3, 3));
    L(3, 0) = L(3, 0) / L(3, 3);
    L(0, 0) = L(0, 0) - L(3, 0) * L(3, 0);
    
    // Joint right_wheel_joint, index 2 :
    L(2, 2) = std::sqrt(L(2, 2));
    L(2, 0) = L(2, 0) / L(2, 2);
    L(0, 0) = L(0, 0) - L(2, 0) * L(2, 0);
    
    // Joint left_wheel_joint, index 1 :
    L(1, 1) = std::sqrt(L(1, 1));
    L(1, 0) = L(1, 0) / L(1, 1);
    L(0, 0) = L(0, 0) - L(1, 0) * L(1, 0);
    
    // Joint chassis_joint, index 0 :
    L(0, 0) = std::sqrt(L(0, 0));
    
}

template <typename TRAIT>
void iit::frea::dyn::tpl::JSIM<TRAIT>::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 0) * Linv(1, 0)) + (Linv(1, 1) * Linv(1, 1));
    inverse(1, 0) =  + (Linv(1, 0) * Linv(0, 0));
    inverse(0, 1) = inverse(1, 0);
    inverse(2, 2) =  + (Linv(2, 0) * Linv(2, 0)) + (Linv(2, 2) * Linv(2, 2));
    inverse(2, 0) =  + (Linv(2, 0) * Linv(0, 0));
    inverse(0, 2) = inverse(2, 0);
    inverse(3, 3) =  + (Linv(3, 0) * Linv(3, 0)) + (Linv(3, 3) * Linv(3, 3));
    inverse(3, 0) =  + (Linv(3, 0) * Linv(0, 0));
    inverse(0, 3) = inverse(3, 0);
    inverse(4, 4) =  + (Linv(4, 0) * Linv(4, 0)) + (Linv(4, 3) * Linv(4, 3)) + (Linv(4, 4) * Linv(4, 4));
    inverse(4, 3) =  + (Linv(4, 0) * Linv(3, 0)) + (Linv(4, 3) * Linv(3, 3));
    inverse(3, 4) = inverse(4, 3);
    inverse(4, 0) =  + (Linv(4, 0) * Linv(0, 0));
    inverse(0, 4) = inverse(4, 0);
    inverse(5, 5) =  + (Linv(5, 0) * Linv(5, 0)) + (Linv(5, 3) * Linv(5, 3)) + (Linv(5, 4) * Linv(5, 4)) + (Linv(5, 5) * Linv(5, 5));
    inverse(5, 4) =  + (Linv(5, 0) * Linv(4, 0)) + (Linv(5, 3) * Linv(4, 3)) + (Linv(5, 4) * Linv(4, 4));
    inverse(4, 5) = inverse(5, 4);
    inverse(5, 3) =  + (Linv(5, 0) * Linv(3, 0)) + (Linv(5, 3) * Linv(3, 3));
    inverse(3, 5) = inverse(5, 3);
    inverse(5, 0) =  + (Linv(5, 0) * Linv(0, 0));
    inverse(0, 5) = inverse(5, 0);
    inverse(6, 6) =  + (Linv(6, 0) * Linv(6, 0)) + (Linv(6, 3) * Linv(6, 3)) + (Linv(6, 4) * Linv(6, 4)) + (Linv(6, 5) * Linv(6, 5)) + (Linv(6, 6) * Linv(6, 6));
    inverse(6, 5) =  + (Linv(6, 0) * Linv(5, 0)) + (Linv(6, 3) * Linv(5, 3)) + (Linv(6, 4) * Linv(5, 4)) + (Linv(6, 5) * Linv(5, 5));
    inverse(5, 6) = inverse(6, 5);
    inverse(6, 4) =  + (Linv(6, 0) * Linv(4, 0)) + (Linv(6, 3) * Linv(4, 3)) + (Linv(6, 4) * Linv(4, 4));
    inverse(4, 6) = inverse(6, 4);
    inverse(6, 3) =  + (Linv(6, 0) * Linv(3, 0)) + (Linv(6, 3) * Linv(3, 3));
    inverse(3, 6) = inverse(6, 3);
    inverse(6, 0) =  + (Linv(6, 0) * Linv(0, 0));
    inverse(0, 6) = inverse(6, 0);
    inverse(7, 7) =  + (Linv(7, 0) * Linv(7, 0)) + (Linv(7, 3) * Linv(7, 3)) + (Linv(7, 4) * Linv(7, 4)) + (Linv(7, 5) * Linv(7, 5)) + (Linv(7, 7) * Linv(7, 7));
    inverse(7, 5) =  + (Linv(7, 0) * Linv(5, 0)) + (Linv(7, 3) * Linv(5, 3)) + (Linv(7, 4) * Linv(5, 4)) + (Linv(7, 5) * Linv(5, 5));
    inverse(5, 7) = inverse(7, 5);
    inverse(7, 4) =  + (Linv(7, 0) * Linv(4, 0)) + (Linv(7, 3) * Linv(4, 3)) + (Linv(7, 4) * Linv(4, 4));
    inverse(4, 7) = inverse(7, 4);
    inverse(7, 3) =  + (Linv(7, 0) * Linv(3, 0)) + (Linv(7, 3) * Linv(3, 3));
    inverse(3, 7) = inverse(7, 3);
    inverse(7, 0) =  + (Linv(7, 0) * Linv(0, 0));
    inverse(0, 7) = inverse(7, 0);
    inverse(8, 8) =  + (Linv(8, 0) * Linv(8, 0)) + (Linv(8, 3) * Linv(8, 3)) + (Linv(8, 4) * Linv(8, 4)) + (Linv(8, 5) * Linv(8, 5)) + (Linv(8, 8) * Linv(8, 8));
    inverse(8, 5) =  + (Linv(8, 0) * Linv(5, 0)) + (Linv(8, 3) * Linv(5, 3)) + (Linv(8, 4) * Linv(5, 4)) + (Linv(8, 5) * Linv(5, 5));
    inverse(5, 8) = inverse(8, 5);
    inverse(8, 4) =  + (Linv(8, 0) * Linv(4, 0)) + (Linv(8, 3) * Linv(4, 3)) + (Linv(8, 4) * Linv(4, 4));
    inverse(4, 8) = inverse(8, 4);
    inverse(8, 3) =  + (Linv(8, 0) * Linv(3, 0)) + (Linv(8, 3) * Linv(3, 3));
    inverse(3, 8) = inverse(8, 3);
    inverse(8, 0) =  + (Linv(8, 0) * Linv(0, 0));
    inverse(0, 8) = inverse(8, 0);
    inverse(9, 9) =  + (Linv(9, 0) * Linv(9, 0)) + (Linv(9, 9) * Linv(9, 9));
    inverse(9, 0) =  + (Linv(9, 0) * Linv(0, 0));
    inverse(0, 9) = inverse(9, 0);
    inverse(10, 10) =  + (Linv(10, 0) * Linv(10, 0)) + (Linv(10, 9) * Linv(10, 9)) + (Linv(10, 10) * Linv(10, 10));
    inverse(10, 9) =  + (Linv(10, 0) * Linv(9, 0)) + (Linv(10, 9) * Linv(9, 9));
    inverse(9, 10) = inverse(10, 9);
    inverse(10, 0) =  + (Linv(10, 0) * Linv(0, 0));
    inverse(0, 10) = inverse(10, 0);
}

template <typename TRAIT>
void iit::frea::dyn::tpl::JSIM<TRAIT>::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
    Linv(4, 4) = 1 / L(4, 4);
    Linv(5, 5) = 1 / L(5, 5);
    Linv(6, 6) = 1 / L(6, 6);
    Linv(7, 7) = 1 / L(7, 7);
    Linv(8, 8) = 1 / L(8, 8);
    Linv(9, 9) = 1 / L(9, 9);
    Linv(10, 10) = 1 / L(10, 10);
    Linv(1, 0) = - Linv(0, 0) * ((Linv(1, 1) * L(1, 0)) + 0);
    Linv(2, 0) = - Linv(0, 0) * ((Linv(2, 2) * L(2, 0)) + 0);
    Linv(3, 0) = - Linv(0, 0) * ((Linv(3, 3) * L(3, 0)) + 0);
    Linv(4, 3) = - Linv(3, 3) * ((Linv(4, 4) * L(4, 3)) + 0);
    Linv(4, 0) = - Linv(0, 0) * ((Linv(4, 3) * L(3, 0)) + (Linv(4, 4) * L(4, 0)) + 0);
    Linv(5, 4) = - Linv(4, 4) * ((Linv(5, 5) * L(5, 4)) + 0);
    Linv(5, 3) = - Linv(3, 3) * ((Linv(5, 4) * L(4, 3)) + (Linv(5, 5) * L(5, 3)) + 0);
    Linv(5, 0) = - Linv(0, 0) * ((Linv(5, 3) * L(3, 0)) + (Linv(5, 4) * L(4, 0)) + (Linv(5, 5) * L(5, 0)) + 0);
    Linv(6, 5) = - Linv(5, 5) * ((Linv(6, 6) * L(6, 5)) + 0);
    Linv(6, 4) = - Linv(4, 4) * ((Linv(6, 5) * L(5, 4)) + (Linv(6, 6) * L(6, 4)) + 0);
    Linv(6, 3) = - Linv(3, 3) * ((Linv(6, 4) * L(4, 3)) + (Linv(6, 5) * L(5, 3)) + (Linv(6, 6) * L(6, 3)) + 0);
    Linv(6, 0) = - Linv(0, 0) * ((Linv(6, 3) * L(3, 0)) + (Linv(6, 4) * L(4, 0)) + (Linv(6, 5) * L(5, 0)) + (Linv(6, 6) * L(6, 0)) + 0);
    Linv(7, 5) = - Linv(5, 5) * ((Linv(7, 7) * L(7, 5)) + 0);
    Linv(7, 4) = - Linv(4, 4) * ((Linv(7, 5) * L(5, 4)) + (Linv(7, 7) * L(7, 4)) + 0);
    Linv(7, 3) = - Linv(3, 3) * ((Linv(7, 4) * L(4, 3)) + (Linv(7, 5) * L(5, 3)) + (Linv(7, 7) * L(7, 3)) + 0);
    Linv(7, 0) = - Linv(0, 0) * ((Linv(7, 3) * L(3, 0)) + (Linv(7, 4) * L(4, 0)) + (Linv(7, 5) * L(5, 0)) + (Linv(7, 7) * L(7, 0)) + 0);
    Linv(8, 5) = - Linv(5, 5) * ((Linv(8, 8) * L(8, 5)) + 0);
    Linv(8, 4) = - Linv(4, 4) * ((Linv(8, 5) * L(5, 4)) + (Linv(8, 8) * L(8, 4)) + 0);
    Linv(8, 3) = - Linv(3, 3) * ((Linv(8, 4) * L(4, 3)) + (Linv(8, 5) * L(5, 3)) + (Linv(8, 8) * L(8, 3)) + 0);
    Linv(8, 0) = - Linv(0, 0) * ((Linv(8, 3) * L(3, 0)) + (Linv(8, 4) * L(4, 0)) + (Linv(8, 5) * L(5, 0)) + (Linv(8, 8) * L(8, 0)) + 0);
    Linv(9, 0) = - Linv(0, 0) * ((Linv(9, 9) * L(9, 0)) + 0);
    Linv(10, 9) = - Linv(9, 9) * ((Linv(10, 10) * L(10, 9)) + 0);
    Linv(10, 0) = - Linv(0, 0) * ((Linv(10, 9) * L(9, 0)) + (Linv(10, 10) * L(10, 0)) + 0);
}

