// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer mRobotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    mRobotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }

    mRobotContainer.mDrivetrain.resetGyro();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Grab the X and Y axis from the left joystick on the controller
    var strafeX = mRobotContainer.mController.getRawAxis(0);
    var forwardY = -mRobotContainer.mController.getRawAxis(1);

    // Right trigger should rotate the robot clockwise, left counterclockwise
    // Add the two [0,1] trigger axes together for a combined period of [-1, 1]
    var rotation = mRobotContainer.mController.getRawAxis(2) + -mRobotContainer.mController.getRawAxis(3);

    mRobotContainer.mDrivetrain.drive(strafeX, forwardY, rotation, true);
  }
}
