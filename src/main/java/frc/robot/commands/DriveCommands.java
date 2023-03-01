// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.DriveMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class DriveCommands {
    public static Command DefaultDriveCommand(CommandJoystick mController, Drivetrain drivetrain,
            SwerveModule[] modules) {
        return Commands.run(() -> {
            var strafeX = MathUtil.applyDeadband(mController.getRawAxis(0), 0.1);
            var forwardY = -MathUtil.applyDeadband(mController.getRawAxis(1), 0.1);
            var rotation = mController.getRawAxis(2) - mController.getRawAxis(3);
            // var strafeX = 0;
            // var forwardY = 1;

            drivetrain.drive(strafeX, forwardY, rotation, true);
        }, drivetrain, modules[0], modules[1], modules[2], modules[3]);
    }

    public static Command resetGyroComamand(Drivetrain driveTrain) {
        return Commands.runOnce(() -> driveTrain.resetGyro(), driveTrain);
    }

    public static Command shiftDriveSpeedCommand(Drivetrain driveTrain) {
        return Commands.runOnce(() -> driveTrain.toggleDriveShifter(), driveTrain);
    }

    public static Command followTrajectoryWithEvent(Drivetrain drivetrain, PathPlannerTrajectory trajectory,
            boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (isFirstPath) {
                        drivetrain.resetOdometry(trajectory.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        trajectory,
                        drivetrain::getPose,
                        new PIDController(DriveMap.kAutonDriveXKp, DriveMap.kAutonDriveXKi, DriveMap.kAutonDriveXKd),
                        new PIDController(DriveMap.kAutonDriveYKp, DriveMap.kAutonDriveYKi, DriveMap.kAutonDriveYKd),
                        new PIDController(DriveMap.kAutonRotationKp, DriveMap.kAutonRotationKi,
                                DriveMap.kAutonRotationKd),
                        drivetrain::drive,
                        drivetrain)

        );

    }

}