package org.team1540.robot.subsystems.feeder;

import static org.team1540.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FeederIOReal implements FeederIO {
    private final TalonFX feederMotor;

    private final StatusSignal<Voltage> feederAppliedVolts;
    private final StatusSignal<Current> feederSupplyCurrent;
    private final StatusSignal<Temperature> feederTempCelsius;
    private final StatusSignal<Current> feederStatorCurrent;

    private final VoltageOut feederVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

    public FeederIOReal(int motorID) {
        feederMotor = new TalonFX(motorID);
        feederAppliedVolts = feederMotor.getMotorVoltage();
        feederSupplyCurrent = feederMotor.getSupplyCurrent();
        feederTempCelsius = feederMotor.getDeviceTemp();
        feederStatorCurrent = feederMotor.getStatorCurrent();

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Apply the configuration to the left motor
        feederMotor.getConfigurator().apply(config);

        // Sets update freqency for the listed values while disabling any other status signals
        BaseStatusSignal.setUpdateFrequencyForAll(50, feederAppliedVolts, feederSupplyCurrent, feederTempCelsius);

        feederMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        BaseStatusSignal.refreshAll(feederAppliedVolts, feederSupplyCurrent, feederStatorCurrent, feederTempCelsius);

        inputs.feederAppliedVolts = feederAppliedVolts.getValueAsDouble();
        inputs.feederSupplyCurrentAmps = feederSupplyCurrent.getValueAsDouble();
        inputs.feederTempCelsius = feederTempCelsius.getValueAsDouble();
        inputs.feederStatorCurrentAmps = feederStatorCurrent.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        feederMotor.setControl(feederVoltageCtrlReq.withOutput(volts));
    }
}
