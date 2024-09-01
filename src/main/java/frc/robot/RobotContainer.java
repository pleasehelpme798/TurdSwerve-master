// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.TurdDrive;
import frc.robot.subsystems.TurdPod;
import frc.robot.subsystems.TurdSwerve;

public class RobotContainer {

  public static final XboxController driver = new XboxController(Constants.driverPort);
  // public static final TurdPod leftPod = new TurdPod(Constants.leftAzimuthID, Constants.leftDriveID, Constants.leftAbsoluteEncoderID, Constants.leftAzimuthInvert,Constants.rightAzimuthInvert, Constants.leftAbsoluteEncoderOffset);
  public static final TurdSwerve swerve = new TurdSwerve();
  

  public RobotContainer() {
    final var Odometry = Shuffleboard.getTab("Odometry");
    configureBindings();
    Supplier<Translation2d> driverRightJoystick = () -> new Translation2d(driver.getRightX(), driver.getRightY());
    Supplier<Translation2d> driverLeftJoystick = () -> new Translation2d(driver.getLeftX(), driver.getLeftY());
    Supplier<Boolean> buttonStart = () -> driver.getStartButton();
    Supplier<Integer> DPAD = () -> driver.getPOV();
    swerve.setDefaultCommand(new TurdDrive(swerve, driverLeftJoystick, driverRightJoystick, buttonStart, DPAD));
    swerve.addDashboardWidgets(Odometry);

  }

  private void configureBindings() {
    // new JoystickButton(driver, 4).onTrue(swerve.resetPods());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
