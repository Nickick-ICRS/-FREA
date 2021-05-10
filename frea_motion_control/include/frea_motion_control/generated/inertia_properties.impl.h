template <typename TRAIT>
iit::frea::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_chassis_link = iit::rbd::Vector3d(-0.02857219,-0.016955871,-1.364242E-12).cast<Scalar>();
    tensor_chassis_link.fill(
        Scalar(5.955353),
        com_chassis_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0685021),
                Scalar(0.063981615),
                Scalar(0.030434182),
                Scalar(0.009341011),
                Scalar(7.534802E-5),
                Scalar(-2.0288478E-4)) );

    com_left_wheel_link = iit::rbd::Vector3d(0.0,-0.0,0.0).cast<Scalar>();
    tensor_left_wheel_link.fill(
        Scalar(0.880493),
        com_left_wheel_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0027470002),
                Scalar(0.0027470002),
                Scalar(0.005285),
                Scalar(0.0),
                Scalar(3.469447E-18),
                Scalar(1.1093949E-10)) );

    com_right_wheel_link = iit::rbd::Vector3d(0.0,0.0,-0.0).cast<Scalar>();
    tensor_right_wheel_link.fill(
        Scalar(0.880493),
        com_right_wheel_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0027470002),
                Scalar(0.0027470002),
                Scalar(0.005285),
                Scalar(0.0),
                Scalar(3.469447E-18),
                Scalar(-1.1093949E-10)) );

    com_lower_neck_link = iit::rbd::Vector3d(0.07747193,-0.0056026895,5.910001E-4).cast<Scalar>();
    tensor_lower_neck_link.fill(
        Scalar(0.919374),
        com_lower_neck_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(8.018275E-4),
                Scalar(0.0066770604),
                Scalar(0.0072118547),
                Scalar(-9.400323E-4),
                Scalar(4.199583E-5),
                Scalar(2.7996333E-7)) );

    com_upper_neck_link = iit::rbd::Vector3d(0.09526179,0.050347593,0.002964).cast<Scalar>();
    tensor_upper_neck_link.fill(
        Scalar(0.099676),
        com_upper_neck_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(5.0379965E-4),
                Scalar(0.0013997592),
                Scalar(0.0017681075),
                Scalar(6.8842736E-4),
                Scalar(3.135359E-6),
                Scalar(4.7171486E-7)) );

    com_head_link = iit::rbd::Vector3d(0.00210326,-0.0030597183,-0.006087).cast<Scalar>();
    tensor_head_link.fill(
        Scalar(0.743802),
        com_head_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0016714908),
                Scalar(0.001338008),
                Scalar(0.0010971267),
                Scalar(7.132841E-7),
                Scalar(2.710335E-4),
                Scalar(9.2603994E-5)) );

    com_mouth_link = iit::rbd::Vector3d(0.046782654,0.023725675,-5.039899E-12).cast<Scalar>();
    tensor_mouth_link.fill(
        Scalar(0.034405),
        com_mouth_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(1.155451E-4),
                Scalar(1.8927106E-4),
                Scalar(1.2154625E-4),
                Scalar(4.7848156E-5),
                Scalar(-4.40055E-8),
                Scalar(-2.78902E-8)) );

    com_left_ear_link = iit::rbd::Vector3d(-0.030220851,-0.047165107,-0.0019260001).cast<Scalar>();
    tensor_left_ear_link.fill(
        Scalar(0.016118),
        com_left_ear_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(4.6581452E-5),
                Scalar(1.905194E-5),
                Scalar(6.543582E-5),
                Scalar(2.7988479E-5),
                Scalar(9.329083E-7),
                Scalar(1.4554256E-6)) );

    com_right_ear_link = iit::rbd::Vector3d(-0.029295221,-0.04556679,9.06E-4).cast<Scalar>();
    tensor_right_ear_link.fill(
        Scalar(0.01533),
        com_right_ear_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(4.1425898E-5),
                Scalar(1.7592796E-5),
                Scalar(5.892653E-5),
                Scalar(2.5713423E-5),
                Scalar(-4.1065962E-7),
                Scalar(-6.4187714E-7)) );

    com_upper_tail_link = iit::rbd::Vector3d(-0.043710805,-0.06420791,5.910001E-4).cast<Scalar>();
    tensor_upper_tail_link.fill(
        Scalar(0.919374),
        com_upper_tail_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0043826364),
                Scalar(0.0030962513),
                Scalar(0.0072118547),
                Scalar(0.0030165466),
                Scalar(-2.0825279E-5),
                Scalar(-3.646975E-5)) );

    com_lower_tail_link = iit::rbd::Vector3d(-0.091275714,0.057257615,0.002964).cast<Scalar>();
    tensor_lower_tail_link.fill(
        Scalar(0.099676),
        com_lower_tail_link,
        rbd::Utils::buildInertiaTensor(
                Scalar(5.7848735E-4),
                Scalar(0.0013250714),
                Scalar(0.0017681076),
                Scalar(-7.3162233E-4),
                Scalar(-1.978047E-6),
                Scalar(2.477983E-6)) );

}

