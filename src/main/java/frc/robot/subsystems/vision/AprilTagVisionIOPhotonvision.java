package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.team6328.Alert;
import frc.robot.RobotStateEstimator;
import frc.robot.util.FieldConstants;

public class AprilTagVisionIOPhotonvision implements AprilTagVisionIO {
    private final PhotonCamera camera;
    private final Transform3d robotToCamera;
    private double poseCacheTimestampSeconds = -1;

    private final Alert disconnectedAlert;

    /**
     * Creates a new VisionIOPhotonVision object.
     *
     * @param cameraName the name of the PhotonVision camera to use; the name must
     *                   be unique
     */
    public AprilTagVisionIOPhotonvision(String cameraName, Transform3d robotToCamera) {
        this.camera = new PhotonCamera(cameraName);
        this.robotToCamera = robotToCamera;
        disconnectedAlert = new Alert("No data from \"" + cameraName + "\"", Alert.AlertType.INFO);
    }

    /**
     * Updates the specified VisionIOInputs object with the latest data from the
     * camera.
     *
     * @param inputs the VisionIOInputs object to update with the latest data from
     *               the camera
     */
    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        PhotonPipelineResult result = camera.getLatestResult();
        poseCacheTimestampSeconds = result.getTimestampSeconds(); // Remember the timestamp of the current result used
        disconnectedAlert.set(!camera.isConnected());
        Optional<EstimatedRobotPose> estimate;

        // Time in the past -- give up, since the following if expects times > 0
        if (result.getTimestampSeconds() < 0) {
            estimate = Optional.empty();
        }
        // If the pose cache timestamp was set, and the result is from the same
        // timestamp, return an empty result
        else if (poseCacheTimestampSeconds > 0
                && Math.abs(poseCacheTimestampSeconds - result.getTimestampSeconds()) < 1e-6) {
            estimate = Optional.empty();
        }
        // If no targets seen, trivial case -- return empty result
        else if (!result.hasTargets()) {
            estimate = Optional.empty();
        } else if (result.getMultiTagResult().estimatedPose.isPresent) {
            var best_tf = result.getMultiTagResult().estimatedPose.best;
            var best = new Pose3d()
                    .plus(best_tf) // field-to-camera
                    .relativeTo(FieldConstants.aprilTags.getOrigin())
                    .plus(robotToCamera.inverse()); // field-to-robot
            estimate = Optional.of(
                    new EstimatedRobotPose(
                            best,
                            result.getTimestampSeconds(),
                            result.getTargets(),
                            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
        } else {
            double smallestRotationDelta = Double.POSITIVE_INFINITY;
            EstimatedRobotPose lowestDeltaPose = null;

            for (PhotonTrackedTarget target : result.targets) {
                int targetFiducialId = target.getFiducialId();
                Optional<Pose3d> targetPosition = FieldConstants.aprilTags.getTagPose(target.getFiducialId());

                if (targetFiducialId == -1 || targetPosition.isEmpty()) // non fiducial target or tag isn't a field tag
                    continue;

                Pose3d altTransformPosition = targetPosition
                        .get()
                        .transformBy(target.getAlternateCameraToTarget().inverse())
                        .transformBy(robotToCamera.inverse());
                Pose3d bestTransformPosition = targetPosition
                        .get()
                        .transformBy(target.getBestCameraToTarget().inverse())
                        .transformBy(robotToCamera.inverse());

                Rotation2d referenceRotation = RobotStateEstimator.getInstance().getEstimatedPose().getRotation();
                double altDifference = Math.abs(
                        referenceRotation.getDegrees()
                                - altTransformPosition.getRotation().toRotation2d().getDegrees());
                double bestDifference = Math.abs(
                        referenceRotation.getDegrees()
                                - bestTransformPosition.getRotation().toRotation2d().getDegrees());

                if (altDifference > 180.0) {
                    altDifference -= 180.0;
                }
                if (bestDifference > 180.0) {
                    bestDifference -= 180.0;
                }

                if (altDifference < smallestRotationDelta) {
                    smallestRotationDelta = altDifference;
                    lowestDeltaPose = new EstimatedRobotPose(
                            altTransformPosition,
                            result.getTimestampSeconds(),
                            result.getTargets(),
                            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
                }
                if (bestDifference < smallestRotationDelta) {
                    smallestRotationDelta = bestDifference;
                    lowestDeltaPose = new EstimatedRobotPose(
                            bestTransformPosition,
                            result.getTimestampSeconds(),
                            result.getTargets(),
                            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
                }
            }
            estimate = Optional.ofNullable(lowestDeltaPose);
        }

        estimate.ifPresent(est -> {
            inputs.estimatedRobotPose = est.estimatedPose;
            inputs.estimatedRobotPoseTimestamp = est.timestampSeconds;
            inputs.latency = result.getLatencyMillis();
            int[] tags = new int[est.targetsUsed.size()];
            for (int i = 0; i < est.targetsUsed.size(); i++) {
                tags[i] = est.targetsUsed.get(i).getFiducialId();
            }
            inputs.tagsSeen = tags;
        });
    }
}