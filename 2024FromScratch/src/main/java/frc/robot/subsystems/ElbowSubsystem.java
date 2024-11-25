// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ElbowSubsystem extends SubsystemBase {
  /** Creates a new ElbowSubsystem. */
  // All hardware classes already have WPILib integration
  final TalonFX m_talonFX = new TalonFX(10);
  final TalonFXSimState m_talonFXSim = m_talonFX.getSimState();


  final TalonFXConfiguration m_talonFXConfig = new TalonFXConfiguration();
  private final PositionVoltage position = new PositionVoltage(0);
  private static final double kGearRatio = 10.0;

  private final DCMotorSim m_motorSimModel = new DCMotorSim(
     LinearSystemId.createDCMotorSystem(
        DCMotor.getKrakenX60Foc(1), 0.001, kGearRatio
     ),
     DCMotor.getKrakenX60Foc(1)
  );

  private double targetPositionRadians = 0;

  InvertedValue m_talonFXInverted = InvertedValue.CounterClockwise_Positive;

  public ElbowSubsystem() {
    var slot1Configs = new Slot1Configs();
    slot1Configs.kV = 0.12;
    slot1Configs.kP = 0.11;
    slot1Configs.kI = 0.48;
    slot1Configs.kD = 0.01;
    m_talonFX.getConfigurator().apply(slot1Configs, 0.050);
   
  }

  @AutoLogOutput
  public double getTargetPosition() {
    return targetPositionRadians;
  }

  @AutoLogOutput
  public double getPosition() {
    return m_talonFX.getPosition().getValueAsDouble();
  }

  public void setTargetPosition(double target) {
    targetPositionRadians = target;
  }

  public void goToPosition(double setpointRadians) {
    position.Slot = 1;
    
    m_talonFX.setControl(position.withPosition(setpointRadians));
   this.setTargetPosition(setpointRadians);
  }

  public void stopElbowMovement() {
    m_talonFX.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
      TalonFXSimState talonFXSim = m_talonFX.getSimState();

      // set the supply voltage of the TalonFX
      talonFXSim.setSupplyVoltage(m_talonFX.getSupplyVoltage().getValueAsDouble());
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
