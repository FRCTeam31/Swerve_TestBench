// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.config.RobotMap;
import frc.robot.prime.drive.swerve.PrimeSwerveDriveTrain;

public class Robot extends TimedRobot {
  private PrimeSwerveDriveTrain m_swerve;
  private Joystick m_controller;

  private final double kJoystickDeadband = 0.15;

  // private TalonSRX Talon1;
  // private WPI_TalonFX Falcon1;
  // private TalonSRX Talon2;
  // private CANSparkMax Spark1;
  // private Solenoid Solenoid1;
  // private Solenoid Solenoid2;
  // private Joystick Joystick1;
  // private Command m_autonomousCommand;

  // private RobotContainer m_robotContainer;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Talon1 = new TalonSRX(RobotMap.TALON_1_ID);
    // Falcon1 = new WPI_TalonFX(RobotMap.FALCON_2_ID);
    // Talon2 = new TalonSRX(RobotMap.TALON_2_ID);
    // Solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.SOLENOID_1_ID);
    // Solenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, RobotMap.SOlENOID_2_ID);
    m_controller = new Joystick(0);
    // Spark1 = new CANSparkMax(RobotMap.SPARK_1_ID, MotorType.kBrushless);
    m_swerve = new PrimeSwerveDriveTrain();
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
    // System.out.println(">>>");
    // System.out.println("FL Encoder offset: " + m_swerve.m_frontLeftModule.m_encoder.getValue());
    // System.out.println("FR Encoder offset: " + m_swerve.m_frontRightModule .m_encoder.getValue());
    // System.out.println("RL Encoder offset: " + m_swerve.m_rearLeftModule.m_encoder.getValue());
    // System.out.println("RR Encoder offset: " + m_swerve.m_rearRightModule.m_encoder.getValue());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }

    m_swerve.resetGyro();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);

    SmartDashboard.putNumber("Front Right Encoder Offset", RobotMap.kFrontLeftEncoderOffset);
    SmartDashboard.putNumber("Front Left Encoder Offset", RobotMap.kFrontRightEncoderOffset);
    SmartDashboard.putNumber("Rear Right Encoder Offset", RobotMap.kRearRightEncoderOffset);
    SmartDashboard.putNumber("Rear Left Encoder Offset", RobotMap.kRearLeftEncoderOffset);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Grab the X and Y axis from the left joystick on the controller
    var strafeX = MathUtil.applyDeadband(m_controller.getRawAxis(0), kJoystickDeadband);
    var forwardY = -1 * MathUtil.applyDeadband(m_controller.getRawAxis(1), kJoystickDeadband);

    // Right trigger should rotate the robot clockwise, left counterclockwise
    // Add the two [0,1] trigger axes together for a combined period of [-1, 1]
    var rotation = MathUtil.applyDeadband((m_controller.getRawAxis(3) + -m_controller.getRawAxis(2)),
      kJoystickDeadband);

    m_swerve.drive(forwardY, strafeX, rotation, false);
  }
}
