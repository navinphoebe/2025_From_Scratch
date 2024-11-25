// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElbowGoToPositionCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.RemoveNoteCommand;
import frc.robot.commands.ReverseShooterAndLogCommand;
import frc.robot.commands.StartIndexingAndLog;
import frc.robot.commands.StartShooterCommand;
import frc.robot.commands.StopShooterCommand;
import frc.robot.commands.WristDefaultCommand;
import frc.robot.commands.WristGoToPositionCommand;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

 public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final VisionSubsystem m_vision = new VisionSubsystem();
  public final WristSubsystem m_wrist = new WristSubsystem();
  public final ElbowSubsystem m_elbow = new ElbowSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driverController.a().whileTrue(new ElbowGoToPositionCommand(m_elbow, 5));
    m_wrist.setDefaultCommand(new WristDefaultCommand(m_wrist));
    m_driverController.b().whileTrue(new WristGoToPositionCommand(m_wrist, 10));
    m_driverController.x().whileTrue(new StartShooterCommand(m_shooter));
    //m_driverController.y().whileTrue(new IntakeNoteCommand(m_intake));
    m_driverController.y().onTrue((new IntakeNoteCommand(m_intake)).alongWith(new StartIndexingAndLog(m_intake)));
    m_driverController.pov(0).whileTrue(new RemoveNoteCommand(m_intake));

   // new Trigger(() -> NoteVisualizer.hasNote()).onTrue((new IntakeNoteCommand(m_intake)).alongWith(new StartIndexingAndLog()));
   new Trigger(() -> Rectangle.isNoteInIndexerBox()).whileTrue((new ReverseShooterAndLogCommand(m_shooter)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  /*public Command getSwerveDriveCommand() {
    return new DrivetrainDefaultCommand(
        m_drivetrain, m_driverController, m_vision);

  } */
}
