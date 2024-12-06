// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class SimulateModel {
    public static double[] m_origin = new double[] { -.26, 0, .2731 };
    public static double[] m_jointArea = new double[] { 0.15, 0, 0 };
    public static double m_armLength = 0.6;
    public static double armAngle = Units.rotationsToRadians(5);
    public static double wristAngle = Units.rotationsToRadians(65);

    public static Pose3d poseA = new Pose3d(-0.26, 0, 0.2731,
            new Rotation3d(Math.toRadians(180), Math.toRadians(armAngle), Math.toRadians(0)));

    public static Pose3d poseB = new Pose3d(0.15, 0, 0,
            new Rotation3d(Math.toRadians(0), Math.toRadians(armAngle + wristAngle), Math.toRadians(-0)));

    public static void getJointDegrees() {
        double x = m_origin[0] + m_armLength * Math.cos(Units.rotationsToRadians(armAngle * -1));
        double z = m_origin[2] + m_armLength * Math.sin(Units.rotationsToRadians(armAngle * -1));
        m_jointArea[0] = x;
        m_jointArea[2] = z;
    }

    public static void setArmAngle(double angle) {
        armAngle = angle;
    }

    public static void setWristAngle(double angle) {
        wristAngle = angle;
    }

    public static void logMechPosition() {
        getJointDegrees();
        poseA = new Pose3d(-0.26, 0, 0.2731,
        new Rotation3d(Math.toRadians(180), Units.rotationsToRadians(armAngle), Math.toRadians(0)));
    poseB = new Pose3d(m_jointArea[0], m_jointArea[1], m_jointArea[2],
        new Rotation3d(Math.toRadians(0), Units.rotationsToRadians(armAngle + wristAngle), Math.toRadians(-0)));
        Logger.recordOutput("Raven Mech", poseA, poseB);
    }
}