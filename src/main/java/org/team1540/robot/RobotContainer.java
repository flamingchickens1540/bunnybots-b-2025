package org.team1540.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.team1540.robot.commands.CharacterizationCommands;
import org.team1540.robot.subsystems.drivetrain.Drivetrain;
import org.team1540.robot.subsystems.feeder.Feeder;
import org.team1540.robot.subsystems.indexer.Indexer;
import org.team1540.robot.subsystems.intake.Intake;
import org.team1540.robot.subsystems.intake.IntakeIO;
import org.team1540.robot.subsystems.intake.IntakeIOTalonFX;
import org.team1540.robot.subsystems.shooter.Shooter;
import org.team1540.robot.subsystems.vision.apriltag.AprilTagVision;
import org.team1540.robot.util.JoystickUtil;

public class RobotContainer {
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Autos");
    private final LoggedNetworkNumber topRPM = new LoggedNetworkNumber("SmartDashboard/Shooter/TopRPM", 4000);
    private final LoggedNetworkNumber bottomRPM = new LoggedNetworkNumber("SmartDashboard/Shooter/BottomRPM", 2000);
    private final LoggedNetworkBoolean useVisionAiming =
            new LoggedNetworkBoolean("SmartDashboard/Drivetrain/UseVisionAiming", true);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    private final Drivetrain drivetrain;
    private final AprilTagVision vision;

    private final Intake intake;
    private final Indexer indexer;
    private final Feeder feeder;
    private final Shooter shooter;

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            drivetrain = Drivetrain.createReal();
            vision = AprilTagVision.createReal();
            intake = new Intake(new IntakeIOTalonFX());
            indexer = Indexer.createReal();
            feeder = Feeder.createReal();
            shooter = Shooter.createReal();
        } else {
            drivetrain = Drivetrain.createDummy();
            vision = AprilTagVision.createDummyIO();
            intake = new Intake(new IntakeIO() {});
            indexer = Indexer.createDummy();
            feeder = Feeder.createDummy();
            shooter = Shooter.createDummy();
        }
        configureButtonBindings();
        configureAutoRoutines();
    }

    private void configureButtonBindings() {
        drivetrain.setDefaultCommand(drivetrain.teleopDriveCommand(driver.getHID(), () -> true));
        driver.start().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientationManual));

        driver.leftTrigger().whileTrue(intake.runCommand(() -> 0.5).alongWith(indexer.runCommand(() -> 0.2)));
        driver.povDown()
                .whileTrue(intake.runCommand(() -> -0.5)
                        .alongWith(indexer.runCommand(() -> -0.2), feeder.runCommand(() -> -0.5, () -> -0.5)));

        Command shootPrepareCommand = Commands.either(
                shooter.commandToSpeed(topRPM::get, bottomRPM::get)
                        .withDeadline(drivetrain.teleopDriveWithHeadingCommand(
                                driver.getHID(), RobotState.getInstance()::getAimingHeading, () -> true)),
                shooter.commandToSpeed(topRPM::get, bottomRPM::get),
                useVisionAiming::get);
        driver.rightBumper().toggleOnTrue(shootPrepareCommand);
        driver.rightTrigger()
                .and(shootPrepareCommand::isScheduled)
                .and(shooter::areFlywheelsSpunUp)
                .whileTrue(feeder.runCommand(() -> 0.5, () -> 0.5));

        new Trigger(shooter::areFlywheelsSpunUp).onTrue(JoystickUtil.rumbleCommand(driver.getHID(), 1, 0.5));

        copilot.povUp().onTrue(Commands.runOnce(() -> {
            topRPM.set(MathUtil.clamp(topRPM.get() + 200, 0, 5500));
            bottomRPM.set(MathUtil.clamp(bottomRPM.get() + 100, 0, 2700));
        }));
        copilot.povDown().onTrue(Commands.runOnce(() -> {
            topRPM.set(MathUtil.clamp(topRPM.get() - 200, 0, 5500));
            bottomRPM.set(MathUtil.clamp(bottomRPM.get() - 100, 0, 2700));
        }));
        DoubleSupplier copilotTriggerAxis = () -> copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis();
        intake.setDefaultCommand(intake.runCommand(copilotTriggerAxis));
        indexer.setDefaultCommand(indexer.runCommand(copilotTriggerAxis));
        feeder.setDefaultCommand(feeder.runCommand(copilotTriggerAxis, copilotTriggerAxis));
    }

    private void configureAutoRoutines() {
        if (Constants.isTuningMode()) {
            autoChooser.addOption(
                    "Shooter FF Char",
                    CharacterizationCommands.feedforward(
                            volts -> shooter.setFlywheelVolts(volts, volts),
                            () -> (1 / 60.0) * (shooter.getTopFlywheelSpeed() + shooter.getBottomFlywheelSpeed()) / 2.0,
                            shooter));
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
