package org.team1540.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOReal implements IndexerIO {
    private final TalonFX spinFalcon;
    private final VoltageOut spinVoltageRequest = new VoltageOut(0);
    private final StatusSignal<AngularVelocity> spinVelocityRPS;
    private final StatusSignal<Voltage> spinMotorAppliedVolts;
    private final StatusSignal<Current> spinSupplyCurrent;
    private final StatusSignal<Current> spinStatorCurrent;
    private final StatusSignal<Temperature> spinTemp;
    private final Debouncer spinConnectedDebounce = new Debouncer(0.5);
    // spinMotorVelocityRPS = 0;
    //        public double spinMotorAppliedVolts = 0;
    //        public double spinSupplyCurrentAmps = 0;
    //        public double spinStatorCurrentAmps = 0;
    //        public boolean spinConnected = true;
    public IndexerIOReal(int id, double motorRatio) {
        spinFalcon = new TalonFX(IndexerConstants.SPIN_MOTOR_ID_POSITIVE);
        spinVelocityRPS = spinFalcon.getVelocity();
        spinMotorAppliedVolts = spinFalcon.getMotorVoltage();
        spinSupplyCurrent = spinFalcon.getSupplyCurrent();
        spinStatorCurrent = spinFalcon.getStatorCurrent();
        spinTemp = spinFalcon.getDeviceTemp();
        TalonFXConfiguration spinTalonFXConfig = new TalonFXConfiguration();

        spinTalonFXConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
        spinTalonFXConfig.CurrentLimits.withStatorCurrentLimit(120);
        spinTalonFXConfig.CurrentLimits.withSupplyCurrentLimit(55);
        spinTalonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        spinTalonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        spinTalonFXConfig.Feedback.SensorToMechanismRatio = motorRatio;
        spinFalcon.getConfigurator().apply(spinTalonFXConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, spinVelocityRPS, spinMotorAppliedVolts, spinSupplyCurrent, spinStatorCurrent, spinTemp);
    }

    @Override
    public void setIndexerVoltage(double voltage) {
        spinFalcon.setControl(spinVoltageRequest.withOutput(voltage));
    }

    @Override
    public void updateInputs(IndexerInputs inputs) {
        StatusCode spinStatus = BaseStatusSignal.refreshAll(
                spinVelocityRPS, spinMotorAppliedVolts, spinSupplyCurrent, spinStatorCurrent, spinTemp);
        inputs.spinConnected = spinConnectedDebounce.calculate(spinStatus.isOK());
        inputs.spinMotorVelocityRPS = spinVelocityRPS.getValueAsDouble();
        inputs.spinMotorAppliedVolts = spinMotorAppliedVolts.getValueAsDouble();
        inputs.spinSupplyCurrentAmps = spinSupplyCurrent.getValueAsDouble();
        inputs.spinStatorCurrentAmps = spinStatorCurrent.getValueAsDouble();
    }

    @Override
    public boolean isConnected() {
        return spinFalcon.isConnected();
    }
}
