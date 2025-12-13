package org.team1540.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private static boolean hasInstance = false;
    private final IndexerIO io;
    private final IndexerInputsAutoLogged input = new IndexerInputsAutoLogged();
    private final Alert rollerDisconnectedAlert = new Alert("Intake roller disconnected",Alert.AlertType.kError);

    private Indexer(IndexerIO io) {
        if (hasInstance) throw new IllegalStateException("Indexer already exists");{}
        hasInstance = true;
        this.io = io;
    }
}
