package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagVisionIO {

    public static class AprilTagVisionIOInputs implements LoggableInputs {
        Pose3d estimatedRobotPose = new Pose3d();
        double estimatedRobotPoseTimestamp = 0.0;
        int[] tagsSeen = new int[] {};
        double lastCameraTimestamp = 0.0;

        @Override
        public void toLog(LogTable table) {
            //table.put("estimatedRobotPose", estimatedRobotPose);
            //table.put("estimatedRobotPoseTimestamp", estimatedRobotPoseTimestamp);
        }

        @Override
        public void fromLog(LogTable table) {
            // TODO Auto-generated method stub
            //throw new UnsupportedOperationException("Unimplemented method 'fromLog'");
        }
    }

    /**
     * Updates the set of loggable inputs.
     *
     * @param inputs the inputs to update
     */
    default void updateInputs(AprilTagVisionIOInputs inputs) {
    }
}
