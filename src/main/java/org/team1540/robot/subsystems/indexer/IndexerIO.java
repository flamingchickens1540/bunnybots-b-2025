package org.team1540.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IntakeInputs {
        public double spinMotorVelocityRPS = 0;
        public double spinMotorAppliedVolts = 0;
        public double spinSupplyCurrentAmps = 0;
        public double spinStatorCurrentAmps = 0;
    }
    default void setIntakeVoltage(double voltage) {}

}
