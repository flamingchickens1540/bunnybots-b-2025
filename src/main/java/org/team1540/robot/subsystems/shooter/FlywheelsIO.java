package org.team1540.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
    @AutoLog
    class FlywheelsIOInputs {
        public double topAppliedVolts = 0.0;
        public double topCurrentAmps = 0.0;
        public double topVelocityRPM = 0.0;
        public double topTempCelsius = 0.0;

        public double bottomAppliedVolts = 0.0;
        public double bottomCurrentAmps = 0.0;
        public double bottomVelocityRPM = 0.0;
        public double bottomTempCelsius = 0.0;
    }

    /**
     * Updates the set of loggable inputs
     */
    default void updateInputs(FlywheelsIOInputs inputs) {}

    /**
     * Runs open loop at the specified voltages
     */
    default void setVoltage(double topVolts, double bottomVolts) {}

    /**
     * Runs closed loop at the specified RPMs
     */
    default void setSpeeds(double topRPM, double bottomRPM) {}

    /**
     * Configures the PID controller
     */
    default void configPID(double kP, double kI, double kD, double kV) {}
}
