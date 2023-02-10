// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.SwerveDriveTrainSubsystem;

public class Robot extends TimedRobot {
  private SwerveDriveTrainSubsystem m_swerve;
  private CommandJoystick m_controller;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_controller = new CommandJoystick(0);
    m_swerve = new SwerveDriveTrainSubsystem();

    m_controller.button(3).onTrue(Commands.runOnce(() -> {
     m_swerve.resetGyro();
     
     System.out.println("[DRIVE] Reset gyro");
    }));

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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Front Right Encoder Value", m_swerve.m_frontRightModule.mEncoder.getRawValue());
    SmartDashboard.putNumber("Front Left Encoder Value", m_swerve.m_frontLeftModule.mEncoder.getRawValue());
    SmartDashboard.putNumber("Rear Right Encoder Value", m_swerve.m_rearRightModule.mEncoder.getRawValue());
    SmartDashboard.putNumber("Rear Left Encoder Value",  m_swerve.m_rearLeftModule.mEncoder.getRawValue());
  }

  @Override
  public void teleopInit() {
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }

    m_swerve.resetGyro();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Grab the X and Y axis from the left joystick on the controller
    var strafeX = m_controller.getRawAxis(0);
    var forwardY = -m_controller.getRawAxis(1);

    // Right trigger should rotate the robot clockwise, left counterclockwise
    // Add the two [0,1] trigger axes together for a combined period of [-1, 1]
    var rotation = m_controller.getRawAxis(2) + -m_controller.getRawAxis(3);

    m_swerve.drive(strafeX, forwardY, rotation, true);
  }
}
