package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionPoseEstimate {
    private final Pose2d estimatedRobotPoseMeters;
    private final double timestampSeconds;
    private final Matrix<N3, N1> visionMeasurementStdevs;

    public VisionPoseEstimate(Pose2d estimatedRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdevs) {
        this.estimatedRobotPoseMeters = estimatedRobotPoseMeters;
        this.timestampSeconds = timestampSeconds;
        this.visionMeasurementStdevs = visionMeasurementStdevs;
    }

    public Pose2d getEstimatedRobotPoseMeters() {
        return estimatedRobotPoseMeters;
    }

    public double getTimestampSeconds() {
        return timestampSeconds;
    }

    public Matrix<N3, N1> getVisionMeasurementStdevs() {
        return visionMeasurementStdevs;
    }
}
