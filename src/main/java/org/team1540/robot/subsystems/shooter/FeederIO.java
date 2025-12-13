package org.team1540.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

    void updateInputs(FeederIOInputs inputs);

    void setVoltage(double volts);

    @AutoLog
    class FeederIOInputs {
        public double feederAppliedVolts = 0.0;
        public double feederSupplyCurrentAmps = 0.0;
        public double feederTempCelsius = 0.0;
        public double feederStatorCurrentAmps = 0.0;

        /**
         * Updates the set of loggable inputs
         */
        void updateInputs(FeederIOInputs inputs) {}

        /**
         * Runs open loop at the specified voltages
         */
        void setVoltage(double volts) {}

        /**
         * Runs closed loop at the specified RPMs
         */
    }
}
