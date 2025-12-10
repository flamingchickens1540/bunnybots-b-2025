package org.team1540.robot.subsystems.drivetrain;


import edu.wpi.first.math.geometry.Rotation2d;

import java.util.concurrent.locks.ReentrantLock;

public class Drivetrain {
    public static ReentrantLock odometryLock = new ReentrantLock();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4];

    private Rotation2d fieldOrrientatedOffset = Rotation2d.kZero;
}
