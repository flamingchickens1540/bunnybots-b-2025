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
        TalonFXConfiguration topConfig = new TalonFXConfiguration();
        TalonFXConfiguration bottomConfig = new TalonFXConfiguration();

        topConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        bottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        bottomConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // shooter current limits are banned. forever.
        topConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        topConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        bottomConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        bottomConfig.CurrentLimits.StatorCurrentLimitEnable = false;

        topConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // topConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        topConfig.Feedback.RotorToSensorRatio = 1.0;
        bottomConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // bottomConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;
        bottomConfig.Feedback.RotorToSensorRatio = 1.0;

        topConfig.Slot0.kP = KP;
        topConfig.Slot0.kI = KI;
        topConfig.Slot0.kD = KD;
        topConfig.Slot0.kS = KS;
        topConfig.Slot0.kV = KV;
        bottomConfig.Slot0.kP = KP;
        bottomConfig.Slot0.kI = KI;
        bottomConfig.Slot0.kD = KD;
        bottomConfig.Slot0.kS = KS;
        bottomConfig.Slot0.kV = KV;

        topMotor.getConfigurator().apply(topConfig);
        bottomMotor.getConfigurator().apply(bottomConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                topVelocity,
                topAppliedVolts,
                topSupplyCurrent,
                topStatorCurrent,
                topTempCelsius,
                bottomVelocity,
                bottomAppliedVolts,
                bottomSupplyCurrent,
                bottomStatorCurrent,
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
                topStatorCurrent,
                topTempCelsius,
                bottomVelocity,
                bottomAppliedVolts,
                bottomSupplyCurrent,
                bottomStatorCurrent,
                bottomTempCelsius);

        inputs.topVelocityRPM = topVelocity.getValueAsDouble() * 60;
        inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
        inputs.topSupplyCurrentAmps = topSupplyCurrent.getValueAsDouble();
        inputs.topStatorCurrentAmps = topStatorCurrent.getValueAsDouble();
        inputs.topTempCelsius = topTempCelsius.getValueAsDouble();

        inputs.bottomVelocityRPM = bottomVelocity.getValueAsDouble() * 60;
        inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
        inputs.bottomSupplyCurrentAmps = bottomSupplyCurrent.getValueAsDouble();
        inputs.bottomStatorCurrentAmps = bottomStatorCurrent.getValueAsDouble();
        inputs.bottomTempCelsius = bottomTempCelsius.getValueAsDouble();
    }

    @Override
    public void setVoltage(double topVolts, double bottomVolts) {
        topMotor.setControl(topVoltageCtrlReq.withOutput(topVolts));
        bottomMotor.setControl(bottomVoltageCtrlReq.withOutput(bottomVolts));
    }

    @Override
    public void setSpeeds(double topRPM, double bottomRPM) {
        topMotor.setControl(topVelocityCtrlReq.withVelocity(topRPM / 60));
        bottomMotor.setControl(bottomVelocityCtrlReq.withVelocity(bottomRPM / 60));
    }

    @Override
    public void configPID(double kP, double kI, double kD, double kV) {
        Slot0Configs pidConfigs = new Slot0Configs();
        topMotor.getConfigurator().refresh(pidConfigs);
        pidConfigs.kP = KP;
        pidConfigs.kI = KI;
        pidConfigs.kD = KD;
        pidConfigs.kV = KV;
        topMotor.getConfigurator().apply(pidConfigs);
        bottomMotor.getConfigurator().apply(pidConfigs);
    }
}
