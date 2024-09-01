// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurdPod extends SubsystemBase {

  private final CANSparkMax azimuth;
  private final CANSparkMax drive;
  private final AnalogEncoder absoluteEncoder;

  private final RelativeEncoder azimuthEncoder;
  private final RelativeEncoder driveEncoder;
  private final SparkPIDController azimuthPID;

  private double azimuthDriveSpeedMultiplier;
  private double speed = 0;
  private double error = 0;
  private double absoluteEncoderOffset;


  public TurdPod(int azimuthID, int driveID, int absoluteEncoderID, boolean azimuthInvert, boolean driveInvert, double absoluteEncoderOffset) {
    azimuth = new CANSparkMax(azimuthID, MotorType.kBrushless);
    drive = new CANSparkMax(driveID, MotorType.kBrushless);
    absoluteEncoder = new AnalogEncoder(absoluteEncoderID);

    azimuthEncoder = azimuth.getEncoder();
    driveEncoder = drive.getEncoder();
    
    azimuth.setInverted(azimuthInvert);
    drive.setInverted(driveInvert);

    driveEncoder.setPositionConversionFactor(Constants.driveMetersPerRotation);
    azimuthEncoder.setPositionConversionFactor(Constants.azimuthRadiansPerRotation);
    absoluteEncoder.setDistancePerRotation(Constants.absoluteEncoderRadiansPerRotation);

    // absoluteEncoder.setPositionOffset(absoluteEncoderOffset);
    this.absoluteEncoderOffset = absoluteEncoderOffset;

    azimuth.setSmartCurrentLimit(Constants.azimuthAmpLimit);
    drive.setSmartCurrentLimit(Constants.driveAmpLimit);

    azimuth.setIdleMode(Constants.azimuthMode);
    drive.setIdleMode(Constants.driveMode);

    azimuthPID = azimuth.getPIDController();

    resetPod();
  }

  public void resetPod() {
    driveEncoder.setPosition(0);
    azimuthEncoder.setPosition(getAbsoluteEncoder());
  }
  public void setPID(double P, double I, double D, double IZone, double outputRange, double ADMult) {
    if (P != azimuthPID.getP()) {azimuthPID.setP(P);}
    if (I != azimuthPID.getI()) {azimuthPID.setI(I);}
    if (D != azimuthPID.getD()) {azimuthPID.setD(D);}
    if (IZone != azimuthPID.getIZone()) {azimuthPID.setIZone(IZone);}
    if (outputRange != azimuthPID.getOutputMax()) {azimuthPID.setOutputRange(-outputRange, outputRange);}
    azimuthPID.setPositionPIDWrappingMaxInput(Math.PI);
    azimuthPID.setPositionPIDWrappingMinInput(-Math.PI);
    azimuthPID.setPositionPIDWrappingEnabled(true);
    // azimuthPID.setSmartMotionAllowedClosedLoopError(0, 0);
    azimuth.setClosedLoopRampRate(0.35);
    azimuthDriveSpeedMultiplier = ADMult;
  }

  public SwerveModulePosition getPodPosition() {
    return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(azimuthEncoder.getPosition()));
  }

  public void setPodState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, new Rotation2d(azimuthEncoder.getPosition())); // does not account for rotations between 180 and 360?
    azimuthPID.setReference(state.angle.getRadians(), ControlType.kPosition); 
    speed = Math.abs(state.speedMetersPerSecond) < .01 ? 0 : state.speedMetersPerSecond * Constants.driveSpeedToPower;
    SmartDashboard.putNumber("state.angle.getRadians()", state.angle.getRadians());



     error = (state.angle.getRadians() - azimuthEncoder.getPosition()) % (2*Math.PI);
      error = error > Math.PI ? error - 2*Math.PI : error;
      error = error < -Math.PI ? error + 2*Math.PI : error;
      error *= 180 / Math.PI;
      SmartDashboard.putNumber("error azimuth " + azimuth.getDeviceId(), error);
  }

  public double getAbsoluteEncoder() {
    return (absoluteEncoder.getAbsolutePosition() * 2*Math.PI) - absoluteEncoderOffset;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("getabsoluteEncoder() " + absoluteEncoder.getChannel(), getAbsoluteEncoder());
    drive.set(speed +  (azimuth.getAppliedOutput() * azimuthDriveSpeedMultiplier));
       azimuth.set(.1);
    SmartDashboard.putNumber("azimuthEncoder.getPosition() " + azimuth.getDeviceId(), azimuthEncoder.getPosition());
    SmartDashboard.putNumber("drive pos " + drive.getDeviceId(), driveEncoder.getPosition());
    SmartDashboard.putNumber("azimuth.getAppliedOutput()" + azimuth.getDeviceId(), azimuth.getAppliedOutput()); //getAppliedOutput());
  }
}
