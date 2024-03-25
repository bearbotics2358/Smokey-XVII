#include <Prefs.h>
#include <frc/apriltag/AprilTagFields.h>
#include "photon/PhotonCamera.h"
#include <photon/PhotonPoseEstimator.h>

class Vision {
    public:
        Vision();
        bool detect_april_tag(int april_tag_fiducial_id);
        std::optional<photon::EstimatedRobotPose>  estimate_position();

    private:
        // For all the vision stuff.
        frc::AprilTagFieldLayout april_tags = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

        // For the photon pose estimator stuff.
        photon::PhotonCamera camera = photon::PhotonCamera(SHOOTER_CAMERA_NAME);
        frc::Transform3d camera_to_robot = frc::Transform3d(
            frc::Translation3d(0.5_m, 0_m, 0.5_m), 
            frc::Rotation3d(0_rad, 25_deg, 0_rad)
        );
        photon::PhotonPoseEstimator estimator = photon::PhotonPoseEstimator(
            april_tags, 
            photon::PoseStrategy::LOWEST_AMBIGUITY,
            camera_to_robot
        );
};