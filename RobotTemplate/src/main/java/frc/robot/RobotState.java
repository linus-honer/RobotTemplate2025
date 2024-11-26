package frc.robot;

import java.util.Map.Entry;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionPoseEstimate;

public class RobotState {
    private final Consumer<VisionPoseEstimate> visionEstimateConsumer;
    private double lastTagTimestamp = 0.0;

    // It is important to have these as atomic because then each step, like get() 
    // or set() occurs in an indivisible step and isnt interfered with by other threads
    private final AtomicReference<ChassisSpeeds> actualRobotRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> actualFieldRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());

    private final AtomicInteger iteration = new AtomicInteger(0);

    private final TimeInterpolatableBuffer<Pose2d> fieldToRobot = TimeInterpolatableBuffer.createBuffer(1.0);

    private TimeInterpolatableBuffer<Double> driveYawAngularVelocity = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
    private TimeInterpolatableBuffer<Double> driveRollAngularVelocity = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
    private TimeInterpolatableBuffer<Double> drivePitchAngularVelocity = TimeInterpolatableBuffer.createDoubleBuffer(1.0);

    private TimeInterpolatableBuffer<Double> drivePitchRads = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
    private TimeInterpolatableBuffer<Double> driveRollRads = TimeInterpolatableBuffer.createDoubleBuffer(1.0);

    private TimeInterpolatableBuffer<Double> accelerationX = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
    private TimeInterpolatableBuffer<Double> accelerationY = TimeInterpolatableBuffer.createDoubleBuffer(1.0);

    // TODO: path canceling stuff

    private double autoStartTimestamp;

    public void setAutoStartTimestamp(double timestamp) {
        autoStartTimestamp = timestamp;
    }

    public double getAutoStartTimestamp() {
        return autoStartTimestamp;
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }
    
    public void incrementIteration() {
        iteration.incrementAndGet();
    }

    public int getIteration() {
        return iteration.get();
    }

    public IntSupplier getIterationSupplier() {
        return () -> getIteration();
    }

    public void addDriveMotionMeasurements(
        double timestamp,
        double rollRadsPerSecond,
        double pitchRadsPerSecond,
        double yawRadsPerSecond,
        double pitchRads,
        double rollRads,
        double accelerationX,
        double accelerationY,
        ChassisSpeeds desiredFieldRelativeSpeeds,
        ChassisSpeeds actualSpeeds,
        ChassisSpeeds actualFieldRelativeSpeeds,
        ChassisSpeeds fusedFieldRelativeSpeeds
    ) {
        this.driveRollAngularVelocity.addSample(timestamp, rollRadsPerSecond);
        this.drivePitchAngularVelocity.addSample(timestamp, pitchRadsPerSecond);
        this.driveYawAngularVelocity.addSample(timestamp, yawRadsPerSecond);
        this.drivePitchRads.addSample(timestamp, pitchRads);
        this.driveRollRads.addSample(timestamp, rollRads);
        this.accelerationX.addSample(timestamp, accelerationX);
        this.accelerationY.addSample(timestamp, accelerationY);
        this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeSpeeds);
        this.actualRobotRelativeChassisSpeeds.set(actualSpeeds);
        this.actualFieldRelativeChassisSpeeds.set(actualFieldRelativeSpeeds);
        this.fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
    }

    public Entry<Double, Pose2d> getLatestFieldToRobot() {
        return fieldToRobot.getInternalBuffer().lastEntry();
    }

    public Pose2d getPredictedFieldToRobot(double lookaheadTime) {
        Entry<Double, Pose2d> predictedFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot = predictedFieldToRobot == null ? new Pose2d(0.0, 0.0, new Rotation2d(0.0)) : predictedFieldToRobot.getValue();
        var delta = getLatestRobotRelativeChassisSpeed();
        delta = delta.times(lookaheadTime);
        return fieldToRobot.exp(new Twist2d(delta.vxMetersPerSecond, delta.vyMetersPerSecond, delta.omegaRadiansPerSecond));
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
        return actualRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
        return desiredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
        return fusedFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedRobotRelativeChassisSpeed() {
        var speeds = getLatestRobotRelativeChassisSpeed();
        speeds.omegaRadiansPerSecond = getLatestFusedFieldRelativeChassisSpeed().omegaRadiansPerSecond;
        return speeds;
    }

    public void updateTagEstimate(VisionPoseEstimate estimate, boolean resetTimestamp) {
        lastTagTimestamp = resetTimestamp ? Timer.getFPGATimestamp() : lastTagTimestamp;
        visionEstimateConsumer.accept(estimate);
    }

    public double lastTagTimestamp() {
        return lastTagTimestamp;
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    public boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Blue));
    }

    public RobotState(Consumer<VisionPoseEstimate> visionEstimateConsumer) {
        this.visionEstimateConsumer = visionEstimateConsumer;

        fieldToRobot.addSample(0.0, new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
        driveYawAngularVelocity.addSample(0.0, 0.0);
    }
}
