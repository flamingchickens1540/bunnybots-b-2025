package org.team1540.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerInputs {
        public double spinMotorVelocityRPS = 0;
        public double spinMotorAppliedVolts = 0;
        public double spinSupplyCurrentAmps = 0;
        public double spinStatorCurrentAmps = 0;
        public boolean spinConnected = true;
    }

    default void setIndexerVoltage(double voltage) {}

    default void updateInputs(IndexerInputs inputs) {}
}
