package org.team1540.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot.Constants;
import org.team1540.robot.Constants.*;
import org.team1540.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private final FlywheelsIO flywheelsIO;
    private final FlywheelsIOInputsAutoLogged flywheelInputs = new FlywheelsIOInputsAutoLogged();

    private double topFlywheelSetpointRPM;
    private double bottomFlywheelSetpointRPM;

    private final LinearFilter topLinearFilter = LinearFilter.movingAverage(20);
    private final LinearFilter bottomLinearFilter = LinearFilter.movingAverage(20);

    private final LoggedTunableNumber flywheelsKP =
            new LoggedTunableNumber("Shooter/Flywheels/kP", ShooterConstants.KP);
    private final LoggedTunableNumber flywheelsKI =
            new LoggedTunableNumber("Shooter/Flywheels/kI", ShooterConstants.KI);
    private final LoggedTunableNumber flywheelsKD =
            new LoggedTunableNumber("Shooter/Flywheels/kD", ShooterConstants.KD);
    private final LoggedTunableNumber flywheelsKV =
            new LoggedTunableNumber("Shooter/Flywheels/kV", ShooterConstants.KV);

    private static boolean hasInstance = false;

    private Shooter(FlywheelsIO flywheelsIO) {
        if (hasInstance) throw new IllegalStateException("Instance of shooter already exists");
        hasInstance = true;
        this.flywheelsIO = flywheelsIO;
    }

    public static Shooter createReal() {
        if (Constants.CURRENT_MODE != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real shooter on simulated robot", false);
        }
        return new Shooter(new FlywheelsIOTalonFX());
    }

    public static Shooter createDummy() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy shooter on real robot", false);
        }
        return new Shooter(new FlywheelsIO() {});
    }

    @Override
    public void periodic() {
        flywheelsIO.updateInputs(flywheelInputs);
        Logger.processInputs("Shooter/Flywheels", flywheelInputs);

        if (RobotState.isDisabled()) {
            stopFlywheels();
        }

        topLinearFilter.calculate(getTopFlywheelSpeed());
        bottomLinearFilter.calculate(getBottomFlywheelSpeed());

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> flywheelsIO.configPID(flywheelsKP.get(), flywheelsKI.get(), flywheelsKD.get(), flywheelsKV.get()),
                flywheelsKP,
                flywheelsKI,
                flywheelsKD,
                flywheelsKV);
    }

    public void stopFlywheels() {
        setFlywheelVolts(0, 0);
    }

    public void setFlywheelSpeeds(double topSpeedRPM, double bottomSpeedRPM) {
        topFlywheelSetpointRPM = topSpeedRPM;
        bottomFlywheelSetpointRPM = bottomSpeedRPM;
        flywheelsIO.setSpeeds(topSpeedRPM, bottomSpeedRPM);
    }

    public void setFlywheelVolts(double topVolts, double bottomVolts) {
        flywheelsIO.setVoltage(MathUtil.clamp(topVolts, -12, 12), MathUtil.clamp(bottomVolts, -12, 12));
    }

    public double getTopFlywheelSpeed() {
        return flywheelInputs.topVelocityRPM;
    }

    public double getBottomFlywheelSpeed() {
        return flywheelInputs.bottomVelocityRPM;
    }

    public double getSpinUpPercent() {
        return ((getTopFlywheelSpeed() + getBottomFlywheelSpeed())
                / (getTopFlywheelSetpointRPM() + getBottomFlywheelSetpointRPM()));
    }

    public boolean areFlywheelsSpunUp() {
        return getSpinUpPercent() >= 0.95;
    }

    @AutoLogOutput(key = "Shooter/Flywheels/topSetpointRPM")
    public double getTopFlywheelSetpointRPM() {
        return topFlywheelSetpointRPM;
    }

    @AutoLogOutput(key = "Shooter/Flywheels/bottomSetpointRPM")
    public double getBottomFlywheelSetpointRPM() {
        return bottomFlywheelSetpointRPM;
    }

    public Command commandToSpeed(DoubleSupplier topRPM, DoubleSupplier bottomRPM) {
        return Commands.runEnd(
                () -> this.setFlywheelSpeeds(topRPM.getAsDouble(), bottomRPM.getAsDouble()),
                () -> this.setFlywheelVolts(0.0, 0.0));
    }
}
