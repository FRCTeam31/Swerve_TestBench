// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class DriveCommands {
    public static Command DefaultDriveCommand(CommandJoystick mController, Drivetrain drivetrain,
            SwerveModule[] modules) {
        return Commands.run(() -> {
            var strafeX = MathUtil.applyDeadband(mController.getRawAxis(0), 0.1);
            var forwardY = -mController.getRawAxis(1);
            var rotation = mController.getRawAxis(2) - mController.getRawAxis(3);

            drivetrain.drive(strafeX, forwardY, rotation, false);
        }, drivetrain, modules[0], modules[1], modules[2], modules[3]);
    }
}
