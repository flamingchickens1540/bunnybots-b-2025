package org.team1540.robot.subsystems.drivetrain;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot.RobotState;
import org.team1540.robot.generated.TunerConstants;

import java.util.concurrent.locks.ReentrantLock;

import static org.team1540.robot.subsystems.drivetrain.DrivetrainConstants.MAX_LINEAR_SPEED_MPS;
import static org.team1540.robot.subsystems.drivetrain.DrivetrainConstants.MAX_STEER_SPEED_RAD_PER_SEC;

public class Drivetrain extends SubsystemBase {
    public static ReentrantLock odometryLock = new ReentrantLock();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];


    private Rotation2d fieldOrrientatedOffset = Rotation2d.kZero;

    // Data stuff
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];
    private double lastOdometryUpdateTime = 0.0;
    private final SwerveDriveKinematics kinematics = RobotState.getInstance().getKinematics();


    @AutoLogOutput(key = "Drivetrain/DesiredSpeeds")
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();

    public Drivetrain(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
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

        Logger.processInputs("Drivetrain-gyro", gyroInputs);

        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
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
                double velocity = moduleDeltas[i].distanceMeters / deltaTime;
                double turnVelocity = modulePositions[moduleIndex]
                        .angle
                        .minus(lastModulePositions[moduleIndex]
                                .angle)
                        .getRadians() / deltaTime;
                if (Math.abs(velocity) > MAX_LINEAR_SPEED_MPS * 2 ||
                        Math.abs(turnVelocity) > MAX_STEER_SPEED_RAD_PER_SEC * 2) {
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
        Logger.recordOutput("OddometryBadData",rejectedSamples);
    }
}




