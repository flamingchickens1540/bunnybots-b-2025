package org.team1540.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;

public class IndexerIOReal implements IndexerIO {
    public TalonFX indexerMotor=new TalonFX(IndexerConstants.indexerMotorID);
    @Override
    public void setIntakeVoltage(double voltage) {
        indexerMotor.setVoltage(voltage);
    }
}
