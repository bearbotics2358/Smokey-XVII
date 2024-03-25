#include <Vision.h>
#include <misc.h>

using namespace photon;

Vision::Vision() {
    april_tags.SetOrigin(
        frc::Pose3d(
            units::meter_t(-0.038),
            units::meter_t(5.55), 
            units::meter_t(1.45), 
            frc::Rotation3d(units::radian_t(0.0), units::radian_t(90.0), units::radian_t(0.0))
        )
    );
}

bool Vision::detect_april_tag(int april_tag_fiducial_id) {
    PhotonPipelineResult result = camera.GetLatestResult();

    if (!result.HasTargets()) {
        return false;
    }

    std::span<const PhotonTrackedTarget> targets = result.GetTargets();
    for (PhotonTrackedTarget target : targets) {
        if (target.GetFiducialId() == april_tag_fiducial_id) {
            return true;
        }
    }
    return false;
}

std::optional<photon::EstimatedRobotPose> Vision::estimate_position() {
    PhotonPipelineResult result = camera.GetLatestResult();

    if (!result.HasTargets()) {
        return std::nullopt;
    }

    EstimatedRobotPose ret = *estimator.Update();
}