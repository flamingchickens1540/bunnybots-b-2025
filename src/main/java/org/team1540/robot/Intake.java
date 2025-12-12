package org.team1540.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private static boolean hasInstance = false;

    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private final Alert intakeDisconnectedAlert = new Alert("Intake roller disconnected", Alert.AlertType.kError);

    private Intake(IntakeIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of intake already exists");
        hasInstance = true;
        this.io = io;
    }

    public void periodic() {
        // LoggedTracer.reset();

        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (DriverStation.isDisabled()) stopAll();

        // LoggedTracer.record("Intake");

        intakeDisconnectedAlert.set(!inputs.intakeConnected);
    }

    public void setIntakeVoltage(double voltage) {
        io.setIntakeVoltage(voltage);
    }

    public void stopAll() {
        setIntakeVoltage(0);
    }

    public Command commandRunIntake(double percent) {
        return Commands.startEnd(() -> this.setIntakeVoltage(percent * 12), () -> this.setIntakeVoltage(0), this);
    }
}
