package org.team1540.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        public double motorVelocityRPM = 0;
        public double motorAppliedVolts = 0;
        public double supplyCurrentAmps = 0;
        public double statorCurrentAmps = 0;
        public double intakeTempCelsius = 0.0;

        public boolean carrotInIntake  = false;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void setIndexerVoltage(double volts) {}

    default void setIndexerVelocity(double velocityRPM) {}

    default void setIndexerBrakeMode(boolean isBrakeMode) {}

}