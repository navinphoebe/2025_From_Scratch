// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.WristSubsystem;

public class WristGoToPositionCommand extends Command {
  private double _targetPosition;
  private WristSubsystem m_wrist;
  /** Creates a new WristGoToPositionCommand. */
  public WristGoToPositionCommand(WristSubsystem wrist, double position) {
    _targetPosition = position;
    m_wrist = wrist;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.goToPosition(_targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.stopWristMovement();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double wristDiff = Math.abs(m_wrist.getPosition() - _targetPosition);
    if (wristDiff <= Constants.WRIST_RANGE_ACCURACY){
      return true;
    }

    return false;
  }
}