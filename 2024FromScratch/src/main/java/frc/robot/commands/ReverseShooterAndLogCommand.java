// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.NoteVisualizer;
import frc.robot.Rectangle;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReverseShooterAndLogCommand extends Command {
  /** Creates a new StartIndexingAndLog. */
  private Timer speedTimer = new Timer();
  private double addX = 0;
  private double addY = 0;
  private double distance;
  private ShooterSubsystem m_shooter;
  public ReverseShooterAndLogCommand(ShooterSubsystem shooter) {
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //speedTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double distance = (m_shooter.getVelocity() / 60) * speedTimer.get() * 2;
    speedTimer.reset();
    speedTimer.start();
    addX = distance * Math.sin(Math.PI - Constants.RADIANS_INDEX_NOTE_POSITION);
    addY = distance * Math.cos(Math.PI - Constants.RADIANS_INDEX_NOTE_POSITION);
    NoteVisualizer.notePosition = new Pose3d(NoteVisualizer.notePosition.getX() - addX, NoteVisualizer.notePosition.getY(), NoteVisualizer.notePosition.getZ() - addY, NoteVisualizer.notePosition.getRotation());
    NoteVisualizer.logHeldNote();
  }




  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Rectangle.isNoteInIndexerBox() == false;
  }
}
