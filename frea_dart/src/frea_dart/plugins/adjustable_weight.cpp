#include "frea_dart/plugins/adjustable_weight.hpp"

AdjustableWeight::AdjustableWeight(
    const dart::dynamics::SkeletonPtr robot, std::string topic_name,
    std::string body_name, Eigen::Vector3d position)
{
    auto bn = robot->getBodyNode(body_name);
    if(!bn) {
        ROS_FATAL_STREAM(
            body_name << " does not exist in " << robot->getName() << "!");
        return;
    }

    // Create a new body attached to the body requested that we change
    // the weight of
    dart::dynamics::WeldJoint::Properties jp;
    jp.mName = bn->getName() + "_weight_joint";
    dart::dynamics::BodyNode::Properties bp;
    bp.mName = bn->getName() + "_weight";
    auto pair =
        bn->createChildJointAndBodyNodePair<dart::dynamics::WeldJoint>(
            jp, bp);
    body_.set(pair.second);
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = position;
    body_->getParentJoint()->setTransformFromParentBodyNode(tf);
    body_->setMass(1e-9);

    weight_sub_ = nh_.subscribe<std_msgs::Float32>(
        topic_name, 1, &AdjustableWeight::changeWeightCallback, this);
}

AdjustableWeight::~AdjustableWeight() {
    // @todo Delete the additional body node we created
}

void AdjustableWeight::update(double /*dt*/, bool reset) {
    if(reset && body_) {
        body_->setMass(1e-9);
    }
}

void AdjustableWeight::changeWeightCallback(
    const std_msgs::Float32ConstPtr &msg)
{
    double mass = msg->data;
    if(mass <= 1e-9) mass = 1e-9;
    body_->setMass(mass);
}
