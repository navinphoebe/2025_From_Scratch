// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.System.Logger;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final TalonFX leader = new TalonFX(20);
  private final TalonFX follower = new TalonFX(2);
  private final VelocityVoltage m_velocity = new VelocityVoltage(0);

  private static final double kGearRatio = 10.0;
  private final DCMotorSim m_motorSimModel = new DCMotorSim(
      LinearSystemId.createDCMotorSystem(
          DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio),
      DCMotor.getKrakenX60Foc(1));

  public ShooterSubsystem() {
    leader.getConfigurator().apply(new TalonFXConfiguration());
    follower.getConfigurator().apply(new TalonFXConfiguration());
    follower.setControl(new Follower(leader.getDeviceID(), false));

    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.11;
    slot0Configs.kI = 0.48;
    slot0Configs.kD = 0.01;
    leader.getConfigurator().apply(slot0Configs, 0.050);
  }

  public void setVelocity(double velocity) {
    leader.setControl(m_velocity.withVelocity(velocity));
  }

  public void stop() {
    leader.stopMotor();
  }

  public void setVoltage(double voltage) {
    leader.setVoltage(0);
  }

  @AutoLogOutput
  public double getVelocity() {
    return leader.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (Robot.isSimulation()) {
      TalonFXSimState talonFXSim = leader.getSimState();

      // set the supply voltage of the TalonFX
      talonFXSim.setSupplyVoltage(leader.getSupplyVoltage().getValueAsDouble());
      // talonFXSim.setSupplyVoltage(12);

      // get the motor voltage of the TalonFX
      var motorVoltage = talonFXSim.getMotorVoltage();
      // use the motor voltage to calculate new position and velocity
      // using WPILib's DCMotorSim class for physics simulation
      m_motorSimModel.setInputVoltage(motorVoltage);
      m_motorSimModel.update(0.020); // assume 20 ms loop time

      // apply the new rotor position and velocity to the TalonFX;
      // note that this is rotor position/velocity (before gear ratio), but
      // DCMotorSim returns mechanism position/velocity (after gear ratio)
      talonFXSim.setRawRotorPosition(
          kGearRatio * m_motorSimModel.getAngularPositionRotations());
      talonFXSim.setRotorVelocity(
          kGearRatio * Units.radiansToRotations(m_motorSimModel.getAngularVelocityRadPerSec()));
    }
  }
}
