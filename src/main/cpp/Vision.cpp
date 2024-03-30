#pragma once

#include <Vision.h>
#include <misc.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace photon;

Vision::Vision() {}

bool Vision::detect_april_tag(int id) {
    PhotonPipelineResult result = camera.GetLatestResult();

    if (!result.HasTargets()) {
        return false;
    }

    std::span<const PhotonTrackedTarget> targets = result.GetTargets();
    for (PhotonTrackedTarget target : targets) {
        if (target.GetFiducialId() == id) {
            return true;
        }
    }
    return false;
}

std::optional<EstimatedRobotPose> Vision::estimate_position() {
    return estimator.Update(camera.GetLatestResult());
}

frc::Pose3d Vision::get_april_tag_pose(int id) {
    return *april_tags.GetTagPose(id);
}