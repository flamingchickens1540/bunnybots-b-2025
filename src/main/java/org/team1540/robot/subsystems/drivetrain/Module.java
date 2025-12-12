package org.team1540.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {

    public enum MountPosition {
        FL(0),
        FR(1),
        BL(2),
        BR(3);

        public final int index;

        MountPosition(int index) {
            this.index = index;
        }
    }

    private final ModuleIO io;
    private final MountPosition mountPosition;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public Module(
            ModuleIO io,
            MountPosition mountPosition,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.io = io;
        this.mountPosition = mountPosition;
        this.constants = constants;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain/module" + mountPosition, inputs);

        odometryPositions = new SwerveModulePosition[inputs.odometryTurnPositions.length];
        for (int i = 0; i < odometryPositions.length; i++) {
            odometryPositions[i] = new SwerveModulePosition(
                    inputs.odometryDrivePositions[i] * constants.WheelRadius, inputs.odometryTurnPositions[i]);
        }
    }

    public void runSetpoint(SwerveModuleState state) {
        state.optimize(inputs.turnPosition);
        state.cosineScale(inputs.turnPosition);
        io.setTurnPosition(state.angle);
        io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
    }

    public void stop() {
        io.setDriveVoltage(0);
        io.setTurnVoltage(0);
    }

    public double getDrivePositionM() {

        return inputs.drivePositionRad * constants.WheelRadius;
    }

    public double getDriveVelocityMPS() {
        return inputs.driveVelocityRadPerSec * constants.WheelRadius;
    }

    public Rotation2d getTurnRotation() {
        return inputs.turnPosition;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionM(), getTurnRotation());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMPS(), getTurnRotation());
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimeStamps;
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }
}
