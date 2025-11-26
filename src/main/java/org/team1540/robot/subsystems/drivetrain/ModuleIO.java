package org.team1540.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.00;
        public double driveVelocityRadPerSec = 0.00;
        public double driveVoltage = 0.00;
        public double driveStatorCurrentAmps = 0.00;
        public double driveSupplyCurrentAmps = 0.00;
        public double driveTempCelsius = 0.00;


        public boolean turnConnected = false;
        public boolean encoderConnected = false;
        public Rotation2d turnPosition = Rotation2d.kZero;
        public Rotation2d encoderPosition = Rotation2d.kZero;
        public double turnVelocityRadPerSec = 0.00;
        public double turnVoltage = 0.00;
        public double turnStatorCurrentAmps = 0.00;
        public double turnSupplyCurrentAmps = 0.00;
        public double turnTempCelsius = 0.00;
        

        public double[] odometryDrivePositions = new double[0];
        public Rotation2d[] odometryTurnPositions = new Rotation2d[0];
        public double[] odometryTimeStamps = new double[0];
        
    }
    default void updateInputs(ModuleIOInputs inputs) {}
    default void setDriveVelocity(double driveVelocityRadPerSec) {}
    default void setTurnPosition(Rotation2d turnPosition) {}
    default void setDriveVoltage(double driveVoltage) {}
    default void setTurnVoltage(double turnVoltage) {}
}
