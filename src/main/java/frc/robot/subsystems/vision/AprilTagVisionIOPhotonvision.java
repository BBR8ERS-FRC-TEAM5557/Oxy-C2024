package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.team6328.Alert;
import frc.robot.util.FieldConstants;

public class AprilTagVisionIOPhotonvision implements AprilTagVisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private double lastTimestamp = 0;

    private static final double disconnectedTimeout = 0.5;
    private final Alert disconnectedAlert;
    private final Timer disconnectedTimer = new Timer();

    /**
     * Creates a new VisionIOPhotonVision object.
     *
     * @param cameraName the name of the PhotonVision camera to use; the name must
     *                   be unique
     */
    public AprilTagVisionIOPhotonvision(String cameraName, Transform3d robotToCamera) {
        this.camera = new PhotonCamera(cameraName);
        this.photonEstimator = new PhotonPoseEstimator(
                FieldConstants.aprilTags,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                robotToCamera);
        //this.photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
        disconnectedAlert = new Alert("No data from \"" + cameraName + "\"", Alert.AlertType.ERROR);
        disconnectedTimer.start();
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
        Optional<EstimatedRobotPose> visionEstimate = this.photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();

        boolean newResult = Math.abs(latestTimestamp - lastTimestamp) > 1e-5;
        if (newResult) {
            visionEstimate.ifPresent(
                    estimate -> {
                        inputs.estimatedRobotPose = estimate.estimatedPose;
                        inputs.estimatedRobotPoseTimestamp = estimate.timestampSeconds;
                        int[] tags = new int[estimate.targetsUsed.size()];
                        for (int i = 0; i < estimate.targetsUsed.size(); i++) {
                            tags[i] = estimate.targetsUsed.get(i).getFiducialId();
                        }
                        inputs.tagsSeen = tags;
                        inputs.lastCameraTimestamp = latestTimestamp;
                        lastTimestamp = latestTimestamp;
                    });
        } else {
            disconnectedTimer.reset();         // Update disconnected alert
        }

        disconnectedAlert.set(disconnectedTimer.hasElapsed(disconnectedTimeout));
    }
}