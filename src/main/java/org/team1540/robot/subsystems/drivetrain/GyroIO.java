package org.team1540.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean connectedGyro = false;
        public Rotation2d yawPosition = Rotation2d.kZero;
        public double yawVelocityRadPerSec = 0.0;
        public Rotation2d[] odometryGyroYaw = new Rotation2d[0];
        public double[] odometryTimeStamps = new double[0];
    }

    default void updateInputs(GyroIOInputs inputs) {}
}
