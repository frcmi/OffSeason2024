package frc.robot.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;

// thank you jack in the bot
public class PhotonlibCamera implements Camera {
    public PhotonlibCamera(String name, Transform3d robotOffset, AprilTagFieldLayout layout) {
        cameraName = name;
        cameraTransform = robotOffset;

        camera = new PhotonCamera(cameraName);
        poseEstimator = new PhotonPoseEstimator(layout, VisionConstants.kPoseStrategy, camera, robotOffset);
        poseEstimator.setMultiTagFallbackStrategy(VisionConstants.kBackupStrategy);
    }

    public void updateResult(Result result) {
        var cameraResult = camera.getLatestResult();
        Optional<EstimatedRobotPose> estimatedPose = isResultValid(cameraResult) ? poseEstimator.update(cameraResult) : Optional.empty();

        List<PhotonTrackedTarget> targets = null;
        if (cameraResult.hasTargets() && estimatedPose.isPresent()) {
            result.hasTarget = true;
            targets = estimatedPose.get().targetsUsed;
        } else {
            result.hasTarget = false;
        }

        result.isNew = false;
        if (targets != null) {
            var firstTarget = targets.get(0);
            result.singleIDUsed = firstTarget.getFiducialId();
            result.pose = estimatedPose.get().estimatedPose.toPose2d();
            result.timestamp = estimatedPose.get().timestampSeconds;

            if (Math.abs(result.timestamp - lastTimestamp) < Double.MIN_VALUE) {
                result.isNew = true;
            }

            if (targets.size() > 1) {
                result.minDistance = Double.MAX_VALUE;
                result.maxDistance = 0;
                result.maxAmbiguity = 0;

                for (var target : targets) {
                    double distance = target.getBestCameraToTarget().getTranslation().getNorm();
                    double ambiguity = target.getPoseAmbiguity();

                    if (distance < result.minDistance) {
                        result.minDistance = distance;
                    }

                    if (distance > result.maxDistance) {
                        result.maxDistance = distance;
                    }

                    if (ambiguity > result.maxAmbiguity) {
                        result.maxAmbiguity = ambiguity;
                    }
                }
            } else {
                result.minDistance = result.maxDistance = firstTarget.getBestCameraToTarget().getTranslation().getNorm();
                result.maxAmbiguity = firstTarget.getPoseAmbiguity();
            }
        }

        lastTimestamp = result.timestamp;
    }

    private boolean isResultValid(PhotonPipelineResult result) {
        for (var target : result.targets) {
            int id = target.getFiducialId();
            if (id < 1 || id > 8) {
                return false;
            }
        }

        return true;
    }

    public String getName() { return cameraName; }
    public Transform3d getTransform() { return cameraTransform; }

    public void setReferencePose(Pose2d pose) {
        poseEstimator.setReferencePose(pose);
    }

    private final String cameraName;
    private final Transform3d cameraTransform;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private double lastTimestamp;
}
