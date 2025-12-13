package org.team1540.robot.subsystems.drivetrain;

import static org.team1540.robot.subsystems.drivetrain.DrivetrainConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot.Constants;
import org.team1540.robot.RobotState;
import org.team1540.robot.generated.TunerConstants;
import org.team1540.robot.util.JoystickUtil;
import org.team1540.robot.util.LoggedTunableNumber;

public class Drivetrain extends SubsystemBase {
    public static ReentrantLock odometryLock = new ReentrantLock();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];

    private Rotation2d fieldOrientatedOffset = Rotation2d.kZero;

    // Data stuff
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];
    private double lastOdometryUpdateTime = 0.0;
    private final SwerveDriveKinematics kinematics = RobotState.getInstance().getKinematics();

    @AutoLogOutput(key = "Drivetrain/DesiredSpeeds")
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    private final LoggedTunableNumber headingKP = new LoggedTunableNumber("Drivetrain/HeadingController/kP", 3.3);
    private final LoggedTunableNumber headingKI = new LoggedTunableNumber("Drivetrain/HeadingController/kI", 0.0);
    private final LoggedTunableNumber headingKD = new LoggedTunableNumber("Drivetrain/HeadingController/kD", 0.3);

    private final ProfiledPIDController headingController = new ProfiledPIDController(
            headingKP.get(),
            headingKI.get(),
            headingKD.get(),
            new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RAD_PER_SEC, 40));

    public Drivetrain(
            GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, Module.MountPosition.FL, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, Module.MountPosition.FR, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, Module.MountPosition.BL, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, Module.MountPosition.BR, TunerConstants.BackRight);

        for (int i = 0; i < 4; i++) {
            lastModulePositions[i] = modules[i].getPosition();
        }

        OdometryThread.getInstance().start();
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        for (Module module : modules) module.periodic();
        odometryLock.unlock();

        Logger.processInputs("Drivetrain/Gyro", gyroInputs);

        double[] sampleTimestamps = gyroInputs.odometryTimeStamps;
        int sampleCount = sampleTimestamps.length;
        int rejectedSamples = 0;

        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
            }
            boolean goodMeasurements = true;
            double deltaTime = sampleTimestamps[i] - lastOdometryUpdateTime;
            for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                double velocity = moduleDeltas[moduleIndex].distanceMeters / deltaTime;
                double turnVelocity = modulePositions[moduleIndex]
                                .angle
                                .minus(lastModulePositions[moduleIndex].angle)
                                .getRadians()
                        / deltaTime;
                if (Math.abs(velocity) > MAX_LINEAR_SPEED_MPS * 2
                        || Math.abs(turnVelocity) > MAX_STEER_SPEED_RAD_PER_SEC * 2) {
                    goodMeasurements = false;
                    break;
                }
            }
            if (goodMeasurements) {
                if (gyroInputs.connectedGyro) rawGyroRotation = gyroInputs.odometryGyroYaw[i];
                else {
                    Twist2d twist = kinematics.toTwist2d(lastModulePositions, modulePositions);
                    rawGyroRotation = rawGyroRotation.plus(Rotation2d.fromRadians(twist.dtheta));
                }
                RobotState.getInstance().addOdometryObservation(modulePositions, rawGyroRotation, sampleTimestamps[i]);
                lastModulePositions = modulePositions;
                lastOdometryUpdateTime = sampleTimestamps[i];
            } else {
                rejectedSamples++;
            }
        }
        Logger.recordOutput("OdometryBadData", rejectedSamples);

        ChassisSpeeds speeds = kinematics.toChassisSpeeds(getModuleStates());
        speeds.omegaRadiansPerSecond =
                gyroInputs.connectedGyro ? gyroInputs.yawVelocityRadPerSec : speeds.omegaRadiansPerSecond;
        RobotState.getInstance().addVelocityData(speeds);

        if (DriverStation.isEnabled()) {
            SwerveModuleState[] setpointStates;
            setpointStates = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.discretize(desiredSpeeds, Constants.LOOP_PERIOD_SECS));
            for (int i = 0; i < 4; i++) {
                modules[i].runSetpoint(setpointStates[i]);
            }
            Logger.recordOutput("Drivetrain/SwerveStates/Setpoints", setpointStates);

        } else {
            for (Module module : modules) module.stop(); // Stop modules when disabled
            Logger.recordOutput(
                    "Drivetrain/SwerveStates/Setpoints",
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState());
        }
    }

    private void runVelocity(ChassisSpeeds speeds) {
        desiredSpeeds = speeds;
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];

        Translation2d[] modulePositions = getModuleTranslations();
        for (int i = 0; i < 4; i++) headings[i] = modulePositions[i].getAngle();
        kinematics.resetHeadings(headings);
        stop();
    }

    public void zeroFieldOrientationManual() {

        fieldOrientatedOffset = rawGyroRotation;
    }

    public void setBrakeMode(boolean enabled) {
        for (Module module : modules) module.setBrakeMode(enabled);
    }

    @AutoLogOutput(key = "Drivetrain/SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public Command percentDriveCommand(
            Supplier<Translation2d> linearPercent, DoubleSupplier omegaPercent, BooleanSupplier fieldRelative) {
        return Commands.run(
                        () -> {
                            var speeds = new ChassisSpeeds(
                                    linearPercent.get().getX() * MAX_LINEAR_SPEED_MPS,
                                    linearPercent.get().getY() * MAX_LINEAR_SPEED_MPS,
                                    omegaPercent.getAsDouble() * MAX_ANGULAR_SPEED_RAD_PER_SEC);
                            if (fieldRelative.getAsBoolean()) {
                                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                        speeds, rawGyroRotation.minus(fieldOrientatedOffset));
                            }
                            runVelocity(speeds);
                        },
                        this)
                .finallyDo(this::stop);
    }

    public Command teleopDriveCommand(XboxController controller, BooleanSupplier fieldRelative) {
        return percentDriveCommand(
                () -> JoystickUtil.deadzonedJoystickTranslation(-controller.getLeftY(), -controller.getLeftX(), 0.15),
                () -> JoystickUtil.smartDeadzone(-controller.getRightX(), 0.15),
                fieldRelative);
    }

    public Command teleopDriveWithHeadingCommand(
            XboxController controller, Supplier<Rotation2d> heading, BooleanSupplier fieldRelative) {
        return percentDriveCommand(
                        () -> JoystickUtil.deadzonedJoystickTranslation(
                                -controller.getLeftY(), -controller.getLeftX(), 0.15),
                        () -> headingController.calculate(
                                        RobotState.getInstance()
                                                .getRobotRotation()
                                                .getRadians(),
                                        new TrapezoidProfile.State(heading.get().getRadians(), 0.0))
                                / MAX_ANGULAR_SPEED_RAD_PER_SEC,
                        fieldRelative)
                .beforeStarting(() -> headingController.reset(
                        RobotState.getInstance().getRobotRotation().getRadians(),
                        RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond))
                .alongWith(Commands.run(() -> Logger.recordOutput("Drivetrain/HeadingGoal", heading.get())))
                .until(() -> Math.abs(controller.getRightX()) >= 0.1);
    }

    public static Drivetrain createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL)
            DriverStation.reportWarning("Using real drivetrain on simulated robot", false);
        return new Drivetrain(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
    }

    public static Drivetrain createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL)
            DriverStation.reportWarning("Using dummy drivetrain on real robot", false);
        return new Drivetrain(
                new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
    }
}
