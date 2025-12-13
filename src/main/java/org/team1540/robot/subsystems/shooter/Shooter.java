package org.team1540.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.team1540.robot.Constants.*;

public class Shooter extends SubsystemBase {
    private final FlywheelsIO flywheelsIO;
    private final FlywheelsIO.FlywheelsIOInputs flywheelInputs = new FlywheelsIO.FlywheelsIOInputs();

    private final FeederIOReal feeder = new FeederIOReal();
    private final FeederIOReal.FeederIOInputs feederInputs = new FeederIO.FeederIOInputs();

    private double topFlywheelSetpointRPM;
    private double bottomFlywheelSetpointRPM;

    private final LinearFilter topLinearFilter = LinearFilter.movingAverage(20);
    private final LinearFilter bottomLinearFilter = LinearFilter.movingAverage(20);

    /*private final LoggedTunableNumber flywheelsKP = new LoggedTunableNumber("Shooter/Flywheels/kP", Flywheels.KP);
    private final LoggedTunableNumber flywheelsKI = new LoggedTunableNumber("Shooter/Flywheels/kI", Flywheels.KI);
    private final LoggedTunableNumber flywheelsKD = new LoggedTunableNumber("Shooter/Flywheels/kD", Flywheels.KD);
    private final LoggedTunableNumber flywheelsKV = new LoggedTunableNumber("Shooter/Flywheels/kV", Flywheels.KV);*/

    private static boolean hasInstance = false;

    private Shooter(FlywheelsIO flywheelsIO) {
        if (hasInstance) throw new IllegalStateException("Instance of shooter already exists");
        hasInstance = true;
        this.flywheelsIO = flywheelsIO;
    }

    /* public static Shooter createReal() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real shooter on simulated robot", false);
        }
        return new Shooter(new ShooterPivotIOTalonFX(), new FlywheelsIOTalonFX());
    }

    public static Shooter createSim() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated shooter on real robot", false);
        }
        return new Shooter(new ShooterPivotIOSim(), new FlywheelsIOSim());
    }

    public static Shooter createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy shooter on real robot", false);
        }
        return new Shooter(new ShooterPivotIO(){}, new FlywheelsIO(){}); */

    @Override
    public void periodic() {
        flywheelsIO.updateInputs(flywheelInputs);
        feeder.updateInputs(feederInputs);
        Logger.processInputs("Shooter/Flywheels/feeder", (LoggableInputs) flywheelInputs);
        Logger.processInputs("Feeder", (LoggableInputs) feederInputs);

        if (RobotState.isDisabled()) {
            stopFlywheels();
            stopFeeder();
        }

        topLinearFilter.calculate(getTopFlywheelSpeed());
        bottomLinearFilter.calculate(getBottomFlywheelSpeed());
    }

    public void stopFlywheels() {
        setFlywheelVolts(0, 0);
    }

    public void stopFeeder() {
        feeder.setVoltage(0);
    }

    // Update tunable numbers
    /*if (Constants.isTuningMode() && (flywheelsKP.hasChanged(hashCode()) || flywheelsKI.hasChanged(hashCode()) || flywheelsKD.hasChanged(hashCode()) || flywheelsKV.hasChanged(hashCode()))) {
    flywheelsIO.configPID(flywheelsKP.get(), flywheelsKI.get(), flywheelsKD.get(), flywheelsKV.get());
    } */
    // Add values to filters

    public void setFlywheelSpeeds(double topSpeedRPM, double bottomSpeedRPM) {
        topFlywheelSetpointRPM = topSpeedRPM;
        bottomFlywheelSetpointRPM = bottomSpeedRPM;
        flywheelsIO.setSpeeds(topSpeedRPM, bottomSpeedRPM);
    }

    public void setFlywheelVolts(double topVolts, double bottomVolts) {
        flywheelsIO.setVoltage(MathUtil.clamp(topVolts, -12, -12), MathUtil.clamp(bottomVolts, -12, 12));
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
        if (getSpinUpPercent() >= 0.95) {
            return true;
        } else {
            return false;
        }
    }

    @AutoLogOutput(key = "Shooter/Flywheels/topSetpointRPM")
    public double getTopFlywheelSetpointRPM() {
        return topFlywheelSetpointRPM;
    }

    @AutoLogOutput(key = "Shooter/Flywheels/bottomSetpointRPM")
    public double getBottomFlywheelSetpointRPM() {
        return bottomFlywheelSetpointRPM;
    }
}
