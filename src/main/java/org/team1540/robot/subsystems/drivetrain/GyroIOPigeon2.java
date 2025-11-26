package org.team1540.robot.subsystems.drivetrain;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import jdk.jshell.Snippet;
import org.team1540.robot.generated.TunerConstants;

import java.util.Queue;

public class GyroIOPigeon2 implements GyroIO{
    private final Pigeon2 gyro = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
    private final StatusSignal<Angle> gyroYaw;

    private final Queue<Double> timeStampQueue;
    private final Queue<Double> yawQueue;
    public GyroIOPigeon2 () {

        gyro.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);

        gyroYaw = gyro.getYaw();

        yawQueue = OdometryThread.getInstance().registerSignal(gyroYaw);
        timeStampQueue = OdometryThread.getInstance().makeTimestampQueue();
    }


}
