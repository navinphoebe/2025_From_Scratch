// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElbowSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ElbowGoToPositionCommand extends Command {
  private double _targetPosition;
  private ElbowSubsystem m_elbow;
  /** Creates a new WristGoToPositionCommand. */
  public ElbowGoToPositionCommand(ElbowSubsystem elbow, double position) {
    _targetPosition = position;
    m_elbow = elbow;
    addRequirements(m_elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elbow.goToPosition(_targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elbow.stopElbowMovement();
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double wristDiff = Math.abs(m_elbow.getPosition() - _targetPosition);
    if (wristDiff <= Constants.ELBOW_RANGE_ACCURACY){
      return true;
    }

    return false;
  }
}