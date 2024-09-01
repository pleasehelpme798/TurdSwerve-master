// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Hashtable;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {
    public static final int leftAzimuthID = 3;
    public static final int rightAzimuthID = 2;

    public static final int leftDriveID = 4;
    public static final int rightDriveID = 1;

    public static final int leftAbsoluteEncoderID = 0;
    public static final int rightAbsoluteEncoderID = 1;

   // public static final int pigeonID = 6;

    public static final double leftAbsoluteEncoderOffset = -1.304;//absolute encoder reading at position
    public static final double rightAbsoluteEncoderOffset = -0.865;// gears facing inwards: fwd/bck TODO: less janky alignment

    public static final boolean leftAzimuthInvert = false;
    public static final boolean rightAzimuthInvert = false;
    public static final boolean leftDriveInvert = false;
    public static final boolean rightDriveInvert = true;

    public static final double azimuthRadiansPerRotation = 2*Math.PI*15/33;
    public static final double driveMetersPerRotation = Units.inchesToMeters(2) * Math.PI * 33 / 45 / 2;
    public static final double absoluteEncoderRadiansPerRotation = 2*Math.PI;
    
    public static final int azimuthAmpLimit = 35;
    public static final int driveAmpLimit = 25;

    public static final double podMaxSpeed = 1;
    public static final double azimuthMaxOutput = 0.25;
    public static double driveSpeedToPower = 0.25;

    public static final double azimuthkP = 0.4;
    public static final double azimuthkI = 0.0;
    public static final double azimuthkD = 0.003;
    public static final double azimuthkIz = 1;
    public static final double azimuthDriveSpeedMultiplier = .5;//0.5;

    public static final IdleMode azimuthMode = IdleMode.kBrake;//originally kBrake

    public static final IdleMode driveMode = IdleMode.kCoast;

    public static final double gyroP = 2;
    public static final double gyroI = 0.0;
    public static final double gyroD = 0.00;

    public static final Translation2d robotCenter = new Translation2d(0, 0); // serves as "center of robot for calculations; robot will turn about this point
    public static final Translation2d leftPodPosition = new Translation2d(-Units.inchesToMeters(9), -Units.inchesToMeters(9)); // units in meters
    public static final Translation2d rightPodPosition = new Translation2d(Units.inchesToMeters(-9), Units.inchesToMeters(-9));
    public static final SwerveDriveKinematics drivetrainKinematics = new SwerveDriveKinematics(robotCenter.minus(leftPodPosition), robotCenter.minus(rightPodPosition));

    public static final int driverPort = 0;

}
