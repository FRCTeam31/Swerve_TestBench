// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands extends CommandBase {
  public static Command runIntake(IntakeSubsystem intakeSubsystem, int direction) {
    return Commands.run(() -> {

      intakeSubsystem.runIntake(0.5 * direction);

    }, intakeSubsystem);

  }

  public static Command stopIntake(IntakeSubsystem intakeSubsystem) {
    return Commands.run(() -> {
      intakeSubsystem.stopIntake();
    }, intakeSubsystem);
  }

  public static Command moveIntake(IntakeSubsystem intakeSubsystem, boolean direction) {
    return Commands.run(() -> {
      intakeSubsystem.moveIntake(direction);
    }, intakeSubsystem);

  }

}
