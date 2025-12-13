package org.team1540.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import org.team1540.robot.generated.TunerConstants;

public class GyroIOPigeon2 implements GyroIO {
    // Gyro
    private final Pigeon2 gyro = new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);

    // Inputs From Gyro
    private final StatusSignal<Angle> gyroYaw;
    private final StatusSignal<AngularVelocity> gyroAngularVelocity;

    // Timestamp
    private final Queue<Double> timeStampQueue;

    // Queue
    private final Queue<Double> yawQueue;

    public GyroIOPigeon2() {
        // Loading Configurations
        gyro.getConfigurator().apply(new Pigeon2Configuration());

        // Zeroing
        gyro.getConfigurator().setYaw(0.0);

        // Recieving Initial Inputs
        gyroYaw = gyro.getYaw();
        gyroAngularVelocity = gyro.getAngularVelocityZWorld();

        // Setting Update Frequency
        gyroYaw.setUpdateFrequency(DrivetrainConstants.ODOMETRY_FREQUENCY_HZ);
        gyroAngularVelocity.setUpdateFrequency(DrivetrainConstants.OTHER_FREQUENCY_HZ);

        // Odometry Queue
        yawQueue = OdometryThread.getInstance().registerSignal(gyroYaw);
        timeStampQueue = OdometryThread.getInstance().makeTimestampQueue();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        BaseStatusSignal.refreshAll(gyroAngularVelocity, gyroYaw);
        inputs.odometryTimeStamps =
                timeStampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryGyroYaw = yawQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        yawQueue.clear();
        timeStampQueue.clear();

        inputs.connectedGyro = gyro.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(gyroYaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyroAngularVelocity.getValueAsDouble());
    }
}
