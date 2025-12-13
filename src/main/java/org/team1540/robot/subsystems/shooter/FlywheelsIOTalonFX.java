package org.team1540.robot.subsystems.shooter;

import static org.team1540.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelsIOTalonFX implements FlywheelsIO {
    private final TalonFX topMotor = new TalonFX(TOP_ID);
    private final TalonFX bottomMotor = new TalonFX(BOTTOM_ID);

    // top motor status signals
    private final StatusSignal<AngularVelocity> topVelocity = topMotor.getVelocity();
    private final StatusSignal<Voltage> topAppliedVolts = topMotor.getMotorVoltage();
    private final StatusSignal<Current> topSupplyCurrent = topMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> topTempCelsius = topMotor.getDeviceTemp();
    private final StatusSignal<Current> topStatorCurrent = topMotor.getStatorCurrent();

    // bottom motor status signals
    private final StatusSignal<AngularVelocity> bottomVelocity = bottomMotor.getVelocity();
    private final StatusSignal<Voltage> bottomAppliedVolts = bottomMotor.getMotorVoltage();
    private final StatusSignal<Current> bottomSupplyCurrent = bottomMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> bottomTempCelsius = bottomMotor.getDeviceTemp();
    private final StatusSignal<Current> bottomStatorCurrent = bottomMotor.getStatorCurrent();

    // basically PID
    private final VelocityVoltage topVelocityCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut topVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

    private final VelocityVoltage bottomVelocityCtrlReq =
            new VelocityVoltage(0).withEnableFOC(true).withSlot(0);
    private final VoltageOut bottomVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

    public FlywheelsIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // shooter current limits are banned. forever.
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // topConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        config.Feedback.RotorToSensorRatio = 1.0;

        config.Slot0.kP = KP;
        config.Slot0.kI = KI;
        config.Slot0.kD = KD;
        config.Slot0.kS = KS;
        config.Slot0.kV = KV;

        topMotor.getConfigurator().apply(config);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        bottomMotor.getConfigurator().apply(config);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                topVelocity,
                topAppliedVolts,
                topSupplyCurrent,
                topTempCelsius,
                bottomVelocity,
                bottomAppliedVolts,
                bottomSupplyCurrent,
                bottomTempCelsius);

        topMotor.optimizeBusUtilization();
        bottomMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                topVelocity,
                topAppliedVolts,
                topSupplyCurrent,
                topTempCelsius,
                bottomVelocity,
                bottomAppliedVolts,
                bottomSupplyCurrent,
                bottomTempCelsius);

        inputs.topVelocityRPM = topVelocity.getValueAsDouble() * 60;
        inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
        inputs.topSupplyCurrentAmps = topSupplyCurrent.getValueAsDouble();
        inputs.topTempCelsius = topTempCelsius.getValueAsDouble();

        inputs.bottomVelocityRPM = topVelocity.getValueAsDouble() * 60;
        inputs.bottomAppliedVolts = topAppliedVolts.getValueAsDouble();
        inputs.bottomSupplyCurrentAmps = topSupplyCurrent.getValueAsDouble();
        inputs.bottomTempCelsius = topTempCelsius.getValueAsDouble();
    }

    @Override
    public void setVoltage(double topVolts, double bottomVolts) {
        topMotor.setControl(topVoltageCtrlReq.withOutput(topVolts));
        bottomMotor.setControl(bottomVoltageCtrlReq.withOutput(bottomVolts));
        // not sure if these will work properly, note to check in with managers
    }

    @Override
    public void setSpeeds(double topRPM, double bottomRPM) {
        topMotor.setControl(topVelocityCtrlReq.withVelocity(topRPM / 60));
        bottomMotor.setControl(bottomVelocityCtrlReq.withVelocity(bottomRPM / 60));
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kV, double kS) {
        Slot0Configs pidConfigs = new Slot0Configs();
        topMotor.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = KP;
        pidConfigs.kI = KI;
        pidConfigs.kD = KD;
        pidConfigs.kV = KV;
        pidConfigs.kS = KS;
        topMotor.getConfigurator().apply(pidConfigs);
        bottomMotor.getConfigurator().apply(pidConfigs);
    }
}
