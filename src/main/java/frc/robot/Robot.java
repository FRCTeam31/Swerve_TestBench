// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.SwerveDriveTrainSubsystem;

public class Robot extends TimedRobot {
  private SwerveDriveTrainSubsystem mSwerve;
  private CommandJoystick mController;

  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    mController = new CommandJoystick(0);
    mSwerve = new SwerveDriveTrainSubsystem();

    mController.button(3).onTrue(Commands.runOnce(() -> {
     mSwerve.resetGyro();
     
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
    SmartDashboard.putNumber("Front Right Encoder Value", mSwerve.mFrontRightModule.mEncoder.getRawValue()); 
    SmartDashboard.putNumber("Front Left Encoder Value", mSwerve.mFrontLeftModule.mEncoder.getRawValue());
    SmartDashboard.putNumber("Rear Right Encoder Value", mSwerve.mRearRightModule.mEncoder.getRawValue());
    SmartDashboard.putNumber("Rear Left Encoder Value",  mSwerve.mRearLeftModule.mEncoder.getRawValue());
  }

  @Override
  public void teleopInit() {
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel(); 
    // }

    mSwerve.resetGyro();  
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

 
}
