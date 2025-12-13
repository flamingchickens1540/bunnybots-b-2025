package org.team1540.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    default void updateInputs(FeederIOInputs inputs) {}

    default void setVoltage(double volts) {}

    @AutoLog
    class FeederIOInputs {
        public double feederAppliedVolts = 0.0;
        public double feederSupplyCurrentAmps = 0.0;
        public double feederTempCelsius = 0.0;
        public double feederStatorCurrentAmps = 0.0;
    }
}
