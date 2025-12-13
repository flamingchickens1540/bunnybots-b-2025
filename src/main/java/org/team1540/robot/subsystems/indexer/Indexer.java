package org.team1540.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private static boolean hasInstance = false;
    private final IndexerIO ioPositive;
    private final IndexerIO ioNegative;
    private final IndexerInputsAutoLogged inputPositive = new IndexerInputsAutoLogged();
    private final IndexerInputsAutoLogged inputNegative = new IndexerInputsAutoLogged();
    private final Alert rollerDisconnectedAlert = new Alert("Intake roller disconnected", Alert.AlertType.kError);

    private Indexer(IndexerIO ioPositive, IndexerIO ioNegative) {
        if (hasInstance) throw new IllegalStateException("Indexer already exists");
        hasInstance = true;
        this.ioPositive = ioPositive;
        this.ioNegative = ioNegative;
    }

    public Command intake() {
        return Commands.run(
                        () -> {
                            ioPositive.setIndexerVoltage(12);
                            ioNegative.setIndexerVoltage(-12);
                        },
                        this)
                .finallyDo(() -> {
                    ioPositive.setIndexerVoltage(0);
                    ioNegative.setIndexerVoltage(0);
                });
    }

    public Command reverseIntake() {
        return Commands.run(
                        () -> {
                            ioPositive.setIndexerVoltage(-12);
                            ioNegative.setIndexerVoltage(12);
                        },
                        this)
                .finallyDo(() -> {
                    ioPositive.setIndexerVoltage(0);
                    ioNegative.setIndexerVoltage(0);
                });
    }

    public Command stop() {
        return Commands.runOnce(() -> {
            ioPositive.setIndexerVoltage(0);
            ioNegative.setIndexerVoltage(0);
        });
    }

    public static Indexer createReal() {
        return new Indexer(
                new IndexerIOReal(IndexerConstants.SPIN_MOTOR_ID_POSITIVE, IndexerConstants.SPIN_GEAR_RATIO_POSITIVE),
                new IndexerIOReal(IndexerConstants.SPIN_MOTOR_ID_NEGATIVE, IndexerConstants.SPIN_GEAR_RATIO_NEGATIVE));
    }

    @Override
    public void periodic() {
        ioNegative.updateInputs(inputNegative);
        ioPositive.updateInputs(inputPositive);
        Logger.processInputs("Indexer/positiveMotor", inputPositive);
        Logger.processInputs("Indexer/negativeMotor", inputNegative);
        if (!ioPositive.isConnected() || !ioNegative.isConnected()) rollerDisconnectedAlert.set(true);
        else rollerDisconnectedAlert.set(false);
    }
}
