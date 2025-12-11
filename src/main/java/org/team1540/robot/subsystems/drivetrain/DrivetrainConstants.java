package org.team1540.robot.subsystems.drivetrain;

import edu.wpi.first.math.system.plant.DCMotor;
import org.team1540.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class DrivetrainConstants {
    public static final double DRIVEBASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    public static final String CAN_BUS = TunerConstants.kCANBus.getName();
    public static final int NORMAL_UPDATE_FREQUENCY_HZ = 50;
    public static final int FAST_UPDATE_FREQUENCY_HZ = 250;

    public static final double MAX_LINEAR_SPEED_MPS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = MAX_LINEAR_SPEED_MPS / DRIVEBASE_RADIUS;
    public static final double MAX_STEER_SPEED_RAD_PER_SEC =   DCMotor.getFalcon500Foc(1).withReduction(TunerConstants.FrontLeft.SteerMotorGearRatio).freeSpeedRadPerSec;


}
