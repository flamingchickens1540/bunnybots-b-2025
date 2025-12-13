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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.team1540.robot.subsystems.drivetrain.DrivetrainConstants;
import org.team1540.robot.subsystems.vision.apriltag.AprilTagVisionConstants;
import org.team1540.robot.subsystems.vision.apriltag.AprilTagVisionIO;
import org.team1540.robot.util.AllianceFlipUtil;

public class RobotState {
    private static final Translation2d TARGET_POSITION =
            new Translation2d(Units.inchesToMeters(148.375), Units.inchesToMeters(158));

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

    public Rotation2d getAimingHeading() {
        Translation2d robotToTarget =
                AllianceFlipUtil.apply(TARGET_POSITION).minus(getEstimatedPose().getTranslation());
        return robotToTarget.getAngle();
    }
}
