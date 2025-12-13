package org.team1540.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX intakeFalcon = new TalonFX(IntakeConstants.DEVICE_ID);
    private final StatusSignal<AngularVelocity> intakeVelocity = intakeFalcon.getVelocity();
    private final StatusSignal<Voltage> intakeAppliedVoltage = intakeFalcon.getMotorVoltage();
    private final StatusSignal<Current> intakeSupplyCurrent = intakeFalcon.getSupplyCurrent();
    private final StatusSignal<Current> intakeStatorCurrent = intakeFalcon.getStatorCurrent();
    private final StatusSignal<Temperature> intakeTemp = intakeFalcon.getDeviceTemp();

    private final VoltageOut intakeVoltageRequest = new VoltageOut(0);

    public IntakeIOTalonFX() {
        TalonFXConfiguration intakeTalonFXConfigs = new TalonFXConfiguration();

        // add more fake constants
        intakeTalonFXConfigs.CurrentLimits.withStatorCurrentLimitEnable(IntakeConstants.IS_ENABLED);
        intakeTalonFXConfigs.CurrentLimits.withStatorCurrentLimit(IntakeConstants.STATOR_LIMIT);
        intakeTalonFXConfigs.CurrentLimits.withSupplyCurrentLimit(IntakeConstants.SUPPLY_LIMIT);

        intakeTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeTalonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        intakeFalcon.getConfigurator().apply(intakeTalonFXConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, intakeVelocity, intakeAppliedVoltage, intakeSupplyCurrent, intakeStatorCurrent, intakeTemp);
        intakeFalcon.optimizeBusUtilization();
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        intakeFalcon.setControl(intakeVoltageRequest.withOutput(voltage));
    }

    public void updateInputs(IntakeInputs inputs) {
        inputs.intakeMotorVelocityRPS = intakeVelocity.getValueAsDouble();
        inputs.intakeMotorAppliedVolts = intakeAppliedVoltage.getValueAsDouble();
        inputs.intakeSupplyCurrentAmps = intakeSupplyCurrent.getValueAsDouble();
        inputs.intakeStatorCurrentAmps = intakeStatorCurrent.getValueAsDouble();
        inputs.intakeTemp = intakeTemp.getValueAsDouble();
    }
}
