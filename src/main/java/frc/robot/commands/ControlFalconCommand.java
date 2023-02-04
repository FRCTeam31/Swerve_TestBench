// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlFalconCommand extends CommandBase {
  // Instance Variables
  private WPI_TalonFX falcon;
  private Joystick joystick;
  private byte axis;
  /** Creates a new ControlFalconCommand. */
  public ControlFalconCommand(WPI_TalonFX falcon, Joystick joystick, byte axis) {
    this.falcon = falcon;
    this.joystick = joystick;
    this.axis = axis;
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    falcon.set(joystick.getRawAxis(axis));
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
