#ifndef OWN_MOTION_MODEL_H
#define OWN_MOTION_MODEL_H

#include <memory>

#include "robot/robot_model.h"
#include "robot/pose.h"
#include "robot/odometry_components.h"
#include "robot/rotary_encoders.h"

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
