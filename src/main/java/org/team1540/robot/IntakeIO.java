package org.team1540.robot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        public boolean intakeConnected = true;
        public double intakeMotorVelocityRPS = 0;
        public double intakeMotorAppliedVolts = 0;
        public double intakeSupplyCurrentAmps = 0;
        public double intakeStatorCurrentAmps = 0;
        public double intakeTemp = 0;
    }

    default void setIntakeVoltage(double voltage) {}

    default void updateInputs(IntakeInputs inputs) {}
}
