// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.SwerveDriveTrainSubsystem;

public class driveCommand extends CommandBase {
  /** Creates a new driveCommand. */
  private SwerveDriveTrainSubsystem mSwerve;
  private CommandJoystick mController;
  private boolean fieldRelative;
  

  public driveCommand(boolean fieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerve = new SwerveDriveTrainSubsystem();
    this.fieldRelative = fieldRelative;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var strafeX = mController.getRawAxis(0);
    var forwardY = mController.getRawAxis(1);
    var rotation = mController.getRawAxis(2) + -mController.getRawAxis(3);
    mSwerve.drive(0, 0, 0, fieldRelative);
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
