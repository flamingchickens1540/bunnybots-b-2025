package org.team1540.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;
import org.team1540.robot.util.swerve.ModuleHWConfigs;

public class ModuleIOTalonFX implements ModuleIO {
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;
    private final TalonFX drive;
    private final TalonFX turn;
    private final CANcoder cancoder;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveSupplyCurrent;
    private final StatusSignal<Current> driveStatorCurrent;
    private final StatusSignal<Temperature> driveTemp;

    // Inputs from turn motor
    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnSupplyCurrent;
    private final StatusSignal<Current> turnStatorCurrent;
    private final StatusSignal<Temperature> turnTemp;

    // Timestamp
    private final Queue<Double> timeStampQueue;

    // Request
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);
    private final PositionVoltage turnPositionRequest = new PositionVoltage(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private final VoltageOut turnVoltageRequest = new VoltageOut(0);

    private final TalonFXConfiguration driveConfig;
    private final TalonFXConfiguration turnConfig;

    public ModuleIOTalonFX(
            // Arguments
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {

        // Implementing Constants
        this.constants = constants;
        ModuleHWConfigs hw = ModuleHWConfigs.fromModuleConstants(constants);
        drive = new TalonFX(constants.DriveMotorId, String.valueOf(constants.DriveMotorId));
        turn = new TalonFX(constants.SteerMotorId, String.valueOf(constants.SteerMotorId));
        cancoder = new CANcoder(constants.EncoderId, String.valueOf(constants.EncoderId));

        driveConfig = hw.driveConfig();
        turnConfig = hw.turnConfig();
        drive.getConfigurator().apply(hw.driveConfig());
        turn.getConfigurator().apply(hw.turnConfig());
        cancoder.getConfigurator().apply(hw.turnEncoderConfig());

        // Recieving Initial Inputs
        drivePosition = drive.getPosition();
        // dzrivePositionQueue = drive.getPosition
        driveVelocity = drive.getVelocity();
        driveAppliedVolts = drive.getMotorVoltage();
        driveSupplyCurrent = drive.getSupplyCurrent();
        driveStatorCurrent = drive.getStatorCurrent();
        driveTemp = drive.getDeviceTemp();

        turnAbsolutePosition = cancoder.getAbsolutePosition();
        turnPosition = turn.getPosition();
        // turnPositionQueue = something
        turnVelocity = turn.getVelocity();
        turnAppliedVolts = turn.getMotorVoltage();
        turnSupplyCurrent = turn.getSupplyCurrent();
        turnStatorCurrent = turn.getStatorCurrent();
        turnTemp = turn.getDeviceTemp();

        // Odometry Queues
        turnPositionQueue = OdometryThread.getInstance().registerSignal(turnPosition);
        drivePositionQueue = OdometryThread.getInstance().registerSignal(drivePosition);
        timeStampQueue = OdometryThread.getInstance().makeTimestampQueue();

        BaseStatusSignal.setUpdateFrequencyForAll(
                DrivetrainConstants.OTHER_FREQUENCY_HZ,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveStatorCurrent,
                driveTemp,
                turnAbsolutePosition,
                turnVelocity,
                turnAppliedVolts,
                turnSupplyCurrent,
                turnStatorCurrent,
                turnTemp);
        BaseStatusSignal.setUpdateFrequencyForAll(
                DrivetrainConstants.ODOMETRY_FREQUENCY_HZ, drivePosition, turnPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Arguments
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveStatorCurrent,
                driveTemp,
                turnAbsolutePosition,
                turnPosition,
                turnVelocity,
                turnAppliedVolts,
                turnSupplyCurrent,
                turnStatorCurrent,
                turnTemp);
        // Drive
        inputs.driveConnected = drive.isConnected();
        inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveVoltage = driveAppliedVolts.getValueAsDouble();
        inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
        inputs.driveStatorCurrentAmps = driveStatorCurrent.getValueAsDouble();
        inputs.driveTempCelsius = driveTemp.getValueAsDouble();

        // Turn
        inputs.turnConnected = turn.isConnected();
        inputs.encoderPosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
        inputs.turnVoltage = turnAppliedVolts.getValueAsDouble();
        inputs.driveSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
        inputs.turnStatorCurrentAmps = turnStatorCurrent.getValueAsDouble();
        inputs.turnTempCelsius = turnTemp.getValueAsDouble();

        inputs.odometryDrivePositions =
                drivePositionQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryTurnPositions =
                turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        inputs.odometryTimeStamps =
                timeStampQueue.stream().mapToDouble(Double::doubleValue).toArray();
    }

    public void setDriveVelocity(double VelocityRadPerSec) {
        drive.setControl(driveVelocityRequest.withVelocity(Units.radiansToRotations(VelocityRadPerSec)));
    }

    @Override
    public void setTurnPosition(Rotation2d turnPosition) {
        turn.setControl(turnPositionRequest.withPosition(turnPosition.getRotations()));
    }

    @Override
    public void setDriveVoltage(double driveVoltage) {
        drive.setControl(driveVoltageRequest.withOutput(driveVoltage));
    }

    @Override
    public void setTurnVoltage(double turnVoltage) {
        turn.setControl(turnVoltageRequest.withOutput(turnVoltage));
    }

    @Override
    public void setDriveVelocity(boolean enabled) {
        driveConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        drive.getConfigurator().apply(driveConfig.MotorOutput);
        turnConfig.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turn.getConfigurator().apply(turnConfig.MotorOutput);
    }
}
