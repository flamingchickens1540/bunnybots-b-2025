package org.team1540.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
    private static boolean hasInstance = false;

    private final FeederIO topIO;
    private final FeederIO bottomIO;
    private final FeederIOInputsAutoLogged topInputs = new FeederIOInputsAutoLogged();
    private final FeederIOInputsAutoLogged bottomInputs = new FeederIOInputsAutoLogged();

    private Feeder(FeederIO topIO, FeederIO bottomIO) {
        if (hasInstance) throw new IllegalStateException("Instance of feeder already exists");
        hasInstance = true;
        this.topIO = topIO;
        this.bottomIO = bottomIO;
    }

    @Override
    public void periodic() {
        topIO.updateInputs(topInputs);
        bottomIO.updateInputs(bottomInputs);
        Logger.processInputs("Feeder/Top", topInputs);
        Logger.processInputs("Feeder/Bottom", bottomInputs);

        if (DriverStation.isDisabled()) {
            topIO.setVoltage(0.0);
            bottomIO.setVoltage(0.0);
        }
    }

    public Command runCommand(DoubleSupplier topSpeed, DoubleSupplier bottomSpeed) {
        return Commands.runEnd(
                () -> {
                    topIO.setVoltage(topSpeed.getAsDouble() * -12);
                    bottomIO.setVoltage(bottomSpeed.getAsDouble() * -12);
                },
                () -> {
                    topIO.setVoltage(0.0);
                    bottomIO.setVoltage(0.0);
                },
                this);
    }

    public static Feeder createReal() {
        return new Feeder(
                new FeederIOReal(FeederConstants.FEEDER_TOP_ID), new FeederIOReal(FeederConstants.FEEDER_BOTTOM_ID));
    }

    public static Feeder createDummy() {
        return new Feeder(new FeederIO() {}, new FeederIO() {});
    }
}
