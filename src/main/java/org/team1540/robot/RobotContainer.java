package org.team1540.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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

public class RobotContainer {
    private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Autos");
    private final LoggedNetworkNumber shooterRPM = new LoggedNetworkNumber("SmartDashboard/Shooter/RPM", 3400);
    private final LoggedNetworkNumber shooterBias = new LoggedNetworkNumber("SmartDashboard/Shooter/RPMBias", 0.69);
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

        DoubleSupplier topRPM = () -> 2 * shooterRPM.get() * shooterBias.get();
        DoubleSupplier bottomRPM = () -> 2 * shooterRPM.get() * (1 - shooterBias.get());

        Command shootPrepareCommand = Commands.either(
                        shooter.commandToSpeed(topRPM, bottomRPM)
                                .withDeadline(drivetrain
                                        .teleopDriveWithHeadingCommand(
                                                driver.getHID(), RobotState.getInstance()::getAimingHeading, () -> true)
                                        .asProxy()),
                        shooter.commandToSpeed(topRPM, bottomRPM),
                        useVisionAiming::get)
                .deadlineFor(Commands.startEnd(
                        () -> driver.setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
                        () -> driver.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)));
        driver.rightStick().toggleOnTrue(shootPrepareCommand);
        driver.rightTrigger()
                .and(shootPrepareCommand::isScheduled)
                .and(shooter::areFlywheelsSpunUp)
                .whileTrue(feeder.runCommand(() -> 0.5, () -> 0.5).alongWith(indexer.runCommand(() -> 0.2)));

        copilot.rightStick()
                .onTrue(Commands.runOnce(() -> shooterRPM.set(MathUtil.clamp(shooterRPM.get() + 100, 0, 5500)))
                        .ignoringDisable(true));
        copilot.leftStick()
                .onTrue(Commands.runOnce(() -> shooterRPM.set(MathUtil.clamp(shooterRPM.get() - 100, 0, 5500)))
                        .ignoringDisable(true));
        copilot.povUp()
                .onTrue(Commands.runOnce(() -> shooterBias.set(MathUtil.clamp(shooterBias.get() + 0.01, 0.5, 0.9)))
                        .ignoringDisable(true));
        copilot.povDown()
                .onTrue(Commands.runOnce(() -> shooterBias.set(MathUtil.clamp(shooterBias.get() - 0.01, 0.5, 0.9)))
                        .ignoringDisable(true));

        DoubleSupplier copilotTriggerAxis = () -> copilot.getRightTriggerAxis() - copilot.getLeftTriggerAxis();
        intake.setDefaultCommand(intake.runCommand(copilotTriggerAxis));
        indexer.setDefaultCommand(indexer.runCommand(copilotTriggerAxis));
        feeder.setDefaultCommand(feeder.runCommand(copilotTriggerAxis, copilotTriggerAxis));

        RobotModeTriggers.teleop().onTrue(Commands.runOnce(drivetrain::zeroFieldOrientation));
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
        autoChooser.addDefaultOption("None", Commands.none());
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
