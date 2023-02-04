// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.config.RobotMap;
import frc.robot.subsystems.TalonSubsystem;

public class ControlTalonCommand extends CommandBase {
  // Instnce Variables
  private TalonSubsystem subSystem;
  private Joystick joystick;

  /** Creates a new ControlTalonCommand. */
  public ControlTalonCommand(TalonSubsystem subSys, Joystick joystick) {
    this.subSystem = subSys;
    this.joystick = joystick;
    this.addRequirements(subSys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var joyAxis = joystick.getRawAxis(RobotMap.TALON1_AXIS);
    subSystem.set(joyAxis);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
