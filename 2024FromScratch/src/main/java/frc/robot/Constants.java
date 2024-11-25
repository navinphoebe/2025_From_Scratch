// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;



  }
  public static final Mode currentMode = Mode.SIM;
  
  public static final double CONTROLLER_DEADBAND_VALUE = .1;
  public static final double WRIST_RANGE_ACCURACY = .5;
  public static final double ELBOW_RANGE_ACCURACY = .5;
  public static final double SHOOTER_TARGET_SPEED_RPM = 95;
 public static final double INTAKE_RAD_PER_MIN = 4;
 public static final double REVERSE_INTAKE_RAD_PER_MIN = -4;
 public static final double WHEEL_CIRCUMFRANCE = Units.inchesToMeters(1.250) * 3.14;
 public static final double RADIANS_INDEX_NOTE_POSITION = 0.75;
 public static final double INDEX_SENSOR_MAX = 0;
 public static final double INDEX_SENSOR_MIN = -100;
 public static final double TILTED_NOTE_RADIUS = Units.inchesToMeters(6);
  
  public static enum Mode { 
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
