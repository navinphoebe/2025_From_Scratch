package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 12;

  public static final double DRIVE_MAX_TURN_RADIANS_PER_SECOND = 7;

  private ChassisSpeeds m_speeds = new ChassisSpeeds(0, 0, 0);
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4445; // The left-to-right distance between the drivetrain wheels
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5461; // The front-to-back distance between the drivetrain wheels.

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front left
    new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0), // Front right
    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0), // Back left
    new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0) // Back right
);
  public DrivetrainSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getMaxVelocity() {
    return MAX_VELOCITY_METERS_PER_SECOND;
  }

  public Pose2d getPose() {
    return new Pose2d();
  }

  public double getMaxAngularVelocity() {
    return DRIVE_MAX_TURN_RADIANS_PER_SECOND;
  }

  public void drive(ChassisSpeeds speeds) {
    m_speeds = speeds;
  }
}
