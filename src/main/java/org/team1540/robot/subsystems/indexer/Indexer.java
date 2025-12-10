package org.team1540.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot.subsystems.indexer.IndexerIO;
import org.team1540.robot.subsystems.indexer.IndexerIOTalonfx;

public class Indexer extends SubsystemBase{
    private final IndexerIO io;
    private final indexerIOInputsAutoLogged inputs = new indexerIoInputsAutoLogged();

    private double feederSetPointRPM = 0.0;

    public static boolean hasInstance = false;
3
    public Indexer(IndexerIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of indexer already exists");
        hasInstance = true;
        this.io = io;
    }

    public static Indexer createReal() {
        /* if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real indexer on simulated robot", false);
        } */
        return new Indexer(new IndexerIOTalonfx())
        }
    }

    public static Indexer createDummy() {

{

    public double spinMotorVelocityRPS = 0;
    public double spinMotorAppliedVolts = 0;
    public double spinSupplyCurrentAmps = 0;
    public double spinStatorCurrentAmps = 0;
}

}
