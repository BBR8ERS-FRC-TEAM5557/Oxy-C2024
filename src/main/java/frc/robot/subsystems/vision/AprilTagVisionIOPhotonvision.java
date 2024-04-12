package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.AprilTagVisionConstants.solveDistCutoff;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.team6328.Alert;
import frc.robot.RobotStateEstimator;
import frc.robot.subsystems.vision.Vision.CalcStrategy;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeometryUtil;

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
        disconnectedAlert.set(!camera.isConnected()); // update alert for if the camera is connected

        // Time in the past -- give up, since the following if expects times > 0
        if (result.getTimestampSeconds() < 0) {
            return;
        }

        // If the pose cache timestamp was set, and the result is from the same
        // timestamp, give up
        if (poseCacheTimestampSeconds > 0
                && Math.abs(poseCacheTimestampSeconds - result.getTimestampSeconds()) < 1e-6) {
            return;
        }

        // If no targets seen, trivial case -- give up
        if (!result.hasTargets()) {
            return;
        }

        // If camera in distance range of cutoff, perform pose calculation using
        // solvePNP
        if (result.getBestTarget().getBestCameraToTarget().getTranslation().toTranslation2d()
                .getNorm() < solveDistCutoff) {
            // If frame has multitag result, calculate using multitag, else use robot
            // rotation to disambiguate
            if (result.getMultiTagResult().estimatedPose.isPresent) {
                var best_tf = result.getMultiTagResult().estimatedPose.best;
                var best = new Pose3d()
                        .plus(best_tf) // field-to-camera
                        .relativeTo(FieldConstants.aprilTags.getOrigin())
                        .plus(robotToCamera.inverse()); // field-to-robot
                inputs.estimatedRobotPose = best;
                inputs.strategy = CalcStrategy.MUTI_SOLVEPNP;
            } else {
                double smallestRotationDelta = Double.POSITIVE_INFINITY;

                for (PhotonTrackedTarget target : result.targets) {
                    int targetFiducialId = target.getFiducialId();
                    Optional<Pose3d> targetPosition = FieldConstants.aprilTags.getTagPose(target.getFiducialId());

                    if (targetFiducialId == -1 || targetPosition.isEmpty()) // non fiducial target or tag isn't a field
                                                                            // tag
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

                    // place within appropriate [0 - 180] scope
                    if (altDifference > 180.0) {
                        altDifference -= 180.0;
                    }
                    if (bestDifference > 180.0) {
                        bestDifference -= 180.0;
                    }

                    if (altDifference < smallestRotationDelta) {
                        smallestRotationDelta = altDifference;
                        inputs.estimatedRobotPose = altTransformPosition;
                    }
                    if (bestDifference < smallestRotationDelta) {
                        smallestRotationDelta = bestDifference;
                        inputs.estimatedRobotPose = bestTransformPosition;
                    }
                    inputs.strategy = CalcStrategy.SINGLE_SOLVEPNP;
                }
            }
        } else {
            double combinedX = 0.0;
            double combinedY = 0.0;

            for (PhotonTrackedTarget target : result.targets) {
                int targetFiducialId = target.getFiducialId();
                Optional<Pose3d> tagPosition = FieldConstants.aprilTags.getTagPose(target.getFiducialId());

                if (targetFiducialId == -1 || tagPosition.isEmpty()) // non fiducial target or tag isn't a field
                                                                     // tag
                    continue;

                Pose2d robotPose = PhotonUtils.estimateFieldToRobot(
                        robotToCamera.getZ(),
                        tagPosition.get().getZ(),
                        robotToCamera.getRotation().getY(),
                        tagPosition.get().getRotation().getY(),
                        Rotation2d.fromDegrees(-target.getYaw()), // target.getYaw is CW Positive but the value needs to be CCW positive
                        // also i don't know if target.getYaw returns radians or degrees so adjust accordingly
                        RobotStateEstimator.getInstance().getEstimatedPose().getRotation(),
                        tagPosition.get().toPose2d(),
                        GeometryUtil.transform3dToTransform2d(robotToCamera.inverse()));

                combinedX += robotPose.getX(); // add to average
                combinedY += robotPose.getY();
            }
            inputs.estimatedRobotPose = new Pose3d(
                    new Pose2d(combinedX / result.targets.size(), combinedY / result.targets.size(),
                            RobotStateEstimator.getInstance().getEstimatedPose().getRotation()));
            // average pose calculation
            inputs.strategy = CalcStrategy.TRIG_AVERAGE;

        }

        // fill in the basic info thats the same regardless of calculation technique
        // avoids useless repetition
        inputs.estimatedRobotPoseTimestamp = result.getTimestampSeconds();
        inputs.latency = result.getLatencyMillis();
        int[] tags = new int[result.getTargets().size()];
        for (int i = 0; i < result.getTargets().size(); i++) {
            tags[i] = result.getTargets().get(i).getFiducialId();
        }
        inputs.tagsSeen = tags;
    }
}
