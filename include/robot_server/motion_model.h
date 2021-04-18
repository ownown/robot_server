#ifndef OWN_MOTION_MODEL_H
#define OWN_MOTION_MODEL_H

#include <memory>

#include "robot_server/robot_model.h"
#include "robot_server/pose.h"
#include "robot_server/odometry_components.h"
#include "robot_server/rotary_encoders.h"

class MotionModel
{
private:
    std::shared_ptr<RobotModel> robot;
    Pose odometry_pose;
    RotaryEncoders rotary_encoder_values;
    OdometryComponents odometry_components;
public:
    void forwardKinematics(const double kLeftEncoderValue, const double kRightEncoderValue);
    MotionModel(std::shared_ptr<RobotModel> robot);
    Pose getOdometryPose();
};

#endif // OWN_MOTION_MODEL_H
