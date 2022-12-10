// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.prime.drive.swerve.PrimeSwerveDriveTrain;
import edu.wpi.first.math.MathUtil;

public class Robot extends TimedRobot {
  private PrimeSwerveDriveTrain m_swerve;
  private XboxController m_controller;

  private final double kJoystickDeadband = 0.15;

  @Override
  public void robotInit() {
    m_controller = new XboxController(0);
    m_swerve = new PrimeSwerveDriveTrain();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    var strafeX = MathUtil.applyDeadband(m_controller.getLeftX(), kJoystickDeadband);
    var forwardY = MathUtil.applyDeadband(m_controller.getLeftY(), kJoystickDeadband);

    // Right trigger should rotate the robot clockwise, left counterclockwise
    var rotation = MathUtil.applyDeadband(
      (-m_controller.getLeftTriggerAxis() + m_controller.getRightTriggerAxis()), // Add the two [0,1] trigger axes together for a combined period of [-1, 1]
      kJoystickDeadband);

    m_swerve.drive(strafeX, forwardY, rotation, fieldRelative);
  }
}
