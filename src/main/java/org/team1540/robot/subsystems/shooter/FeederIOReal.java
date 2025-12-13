package org.team1540.robot.subsystems.shooter;

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
    private final TalonFX feederMotor = new TalonFX(FEEDER_ID);

    private final StatusSignal<Voltage> feederAppliedVolts = feederMotor.getMotorVoltage();
    private final StatusSignal<Current> feederSupplyCurrent = feederMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> feederTempCelsius = feederMotor.getDeviceTemp();
    private final StatusSignal<Current> feederStatorCurrent = feederMotor.getStatorCurrent();

    private final VoltageOut feederVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

    public FeederIOReal() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // TODO: finsh config with voltage limits and feedback values

        config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Sets neutral mode (no input) to coast
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

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
