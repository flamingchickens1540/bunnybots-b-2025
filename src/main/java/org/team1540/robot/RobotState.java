package org.team1540.robot;

import static org.team1540.robot.subsystems.vision.apriltag.AprilTagVisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot.subsystems.drivetrain.DrivetrainConstants;
import org.team1540.robot.subsystems.vision.apriltag.AprilTagVisionConstants;
import org.team1540.robot.subsystems.vision.apriltag.AprilTagVisionIO;

public class RobotState {
    private static RobotState instance = null;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(DrivetrainConstants.getModuleTranslations());
    private final SwerveDrivePoseEstimator poseEstimator;
    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private Pose2d odometryPose = Pose2d.kZero;

    private final Timer resetTimer = new Timer();

    private Rotation2d lastGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()
    };

    private Pose2d[] activeTrajectory;

    private final Field2d field = new Field2d();

    private RobotState() {
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                lastGyroRotation,
                lastModulePositions,
                Pose2d.kZero,
                VecBuilder.fill(0.05, 0.05, 0.1),
                VecBuilder.fill(0.5, 0.5, 5.0));
        resetTimer.start();

        SmartDashboard.putData(field);

        AutoLogOutputManager.addObject(this);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void addOdometryObservation(SwerveModulePosition[] modulePositions, Rotation2d gyroAngle, double timestamp) {
        Twist2d twist = kinematics.toTwist2d(lastModulePositions, modulePositions);
        lastModulePositions = modulePositions;
        lastGyroRotation = gyroAngle;

        odometryPose = odometryPose.exp(twist);
        odometryPose = new Pose2d(odometryPose.getTranslation(), gyroAngle);

        poseEstimator.updateWithTime(timestamp, gyroAngle, modulePositions);
        field.setRobotPose(getEstimatedPose());
    }

    public boolean addVisionMeasurement(AprilTagVisionIO.PoseObservation visionPose) {
        if (shouldAcceptVision(visionPose) && resetTimer.hasElapsed(0.1)) {
            poseEstimator.addVisionMeasurement(
                    visionPose.estimatedPoseMeters().toPose2d(), visionPose.timestampSecs(), getStdDevs(visionPose));
            return true;
        }
        return false;
    }

    double XY_STD_DEV_COEFF = 0.15;
    double ROT_STD_DEV_COEFF = 0.25;

    private Matrix<N3, N1> getStdDevs(AprilTagVisionIO.PoseObservation poseObservation) {
        double xyStdDev =
                XY_STD_DEV_COEFF * Math.pow(poseObservation.avgTagDistance(), 2.0) / poseObservation.numTagsSeen();
        double rotStdDev =
                ROT_STD_DEV_COEFF * Math.pow(poseObservation.avgTagDistance(), 2.0) / poseObservation.numTagsSeen();
        return VecBuilder.fill(
                xyStdDev,
                xyStdDev,
                DriverStation.isEnabled() && poseObservation.numTagsSeen() <= 1 ? Double.POSITIVE_INFINITY : rotStdDev);
    }

    int MIN_ACCEPTED_NUM_TAGS = 1;
    final double MAX_AMBIGUITY = 0.2;
    double MAX_OUTSIDE_OF_FIELD_TOLERANCE = 0.1;

    private boolean shouldAcceptVision(AprilTagVisionIO.PoseObservation poseObservation) {
        Pose3d estimatedPose = poseObservation.estimatedPoseMeters();
        return poseObservation.numTagsSeen() >= MIN_ACCEPTED_NUM_TAGS // Must see sufficient tags
                && (poseObservation.numTagsSeen() > 1
                        || poseObservation.ambiguity() < MAX_AMBIGUITY) // Must be multiple tags or low ambiguity
                // Must be within field roughly
                && estimatedPose.getX() >= -MAX_OUTSIDE_OF_FIELD_TOLERANCE
                && estimatedPose.getX()
                        <= AprilTagVisionConstants.FieldConstants.aprilTagFieldLayout.getFieldLength()
                                + MAX_OUTSIDE_OF_FIELD_TOLERANCE
                && estimatedPose.getY() >= -MAX_OUTSIDE_OF_FIELD_TOLERANCE
                && estimatedPose.getY()
                        <= AprilTagVisionConstants.FieldConstants.aprilTagFieldLayout.getFieldWidth()
                                + MAX_OUTSIDE_OF_FIELD_TOLERANCE
                // Must not be actively flying
                && Math.abs(estimatedPose.getZ()) <= MAX_ROBOT_Z_TOLERANCE;
    }

    public void addVelocityData(ChassisSpeeds velocity) {
        robotVelocity = velocity;
    }

    public void setActiveTrajectory(Pose2d... poses) {
        activeTrajectory = poses;
        field.getObject("trajectory").setPoses(activeTrajectory);
        Logger.recordOutput("Odometry/Trajectory/ActiveTrajectory", activeTrajectory);
    }

    public void clearActiveTrajectory() {
        field.getObject("trajectory").setPoses();
        Logger.recordOutput("Odometry/Trajectory/ActiveTrajectory", new Pose2d[0]);
        activeTrajectory = null;
    }

    public void setTrajectoryTarget(Pose2d target) {
        Logger.recordOutput("Odometry/Trajectory/TargetPose", target);
    }

    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPosition(lastGyroRotation, lastModulePositions, newPose);
        odometryPose = newPose;
        resetTimer.restart();
    }

    @AutoLogOutput(key = "Odometry/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRobotRotation() {
        return getEstimatedPose().getRotation();
    }

    @AutoLogOutput(key = "Odometry/RobotVelocity")
    public ChassisSpeeds getRobotVelocity() {
        return robotVelocity;
    }

    @AutoLogOutput(key = "Odometry/RobotVelocityMagnitude")
    public double getRobotVelocityMagnitude() {
        return Math.sqrt(
                Math.pow(getRobotVelocity().vxMetersPerSecond, 2) + Math.pow(getRobotVelocity().vyMetersPerSecond, 2));
    }

    @AutoLogOutput(key = "Odometry/FieldRelativeVelocity")
    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotVelocity(), getRobotRotation());
    }
}
