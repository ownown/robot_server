#include "robot/motion_model.h"

#include <cmath>
#include <memory>

#include "robot/robot_model.h"
#include "robot/pose.h"

MotionModel::MotionModel(std::shared_ptr<RobotModel> robot) :
    robot(robot), odometry_pose({0.0, 0.0, 0.0}), rotary_encoder_values({0.0,0.0}),
    odometry_components({0.0,0.0,0.0})
{

}

void MotionModel::forwardKinematics(
    const double kLeftEncoderValue,
    const double kRightEncoderValue
)
{
    Pose old_pose = this->odometry_pose;
    // The change in the motor values. As we have the robot resetting the
    // encoder positions on each read, we don't need to interact with our
    // recorded positions.
    const double kDeltaLeft = kLeftEncoderValue * this->robot->getWheelRadius();
    const double kDeltaRight = kRightEncoderValue * this->robot->getWheelRadius();

    double new_x = old_pose.getX();
    double new_y = old_pose.getY();
    double new_theta = old_pose.getTheta();

    const double kTimestep = 1.0;

    if (kDeltaLeft == kDeltaRight)
    {
        // If the distance travelled in both wheels is the same, then the robot
        // either didn't move or moved straight forward. If it didn't move then
        // that is covered already by initialising the new values to the old
        // ones.
        if (kDeltaLeft != 0.0)
        {
            const double kSpeed = (kDeltaLeft + kDeltaRight) / 2.0;
            const double kDistance = kSpeed * kTimestep;

            /**
             * A little bit of trig
             *
             * x' = x + d*cos(\theta)
             * y' = y + d*sin(\theta)
             *
             * Simplified because we've already assigned the old values to the
             * new_ variables
             *
             */
            new_x += kDistance * std::cos(new_theta);
            new_y += kDistance * std::sin(new_theta);
        }
    }
    else
    {
        // If there is a different between the wheels, then we need to account
        // for rotational movement too

        // Angular momentum
        const double kOmega = (kDeltaLeft - kDeltaRight) / this->robot->getAxleLength();

        // Angular momentum over a given timestep
        const double kOmegaDeltaTime = kOmega * kTimestep;


        const double kICRRadius = (this->robot->getAxleLength() / 2.0) *
            ((kDeltaLeft + kDeltaRight) / (kDeltaRight - kDeltaLeft));
        const double kICRX = new_x - (kICRRadius * std::sin(new_theta));
        const double kICRY = new_y + (kICRRadius * std::cos(new_theta));

        new_x = (std::cos(kOmegaDeltaTime) * (old_pose.getX() - kICRX)) +
                ((-std::sin(kOmegaDeltaTime)) * (old_pose.getY() - kICRY)) +
                kICRX;
        new_y = (std::sin(kOmegaDeltaTime) * (old_pose.getX() - kICRX)) +
                (std::cos(kOmegaDeltaTime) * (old_pose.getY() - kICRY)) +
                kICRY;
        new_theta += kOmegaDeltaTime;
    }
    this->odometry_pose = {new_x, new_y, new_theta};
}

Pose MotionModel::getOdometryPose()
{
    return this->odometry_pose;
}