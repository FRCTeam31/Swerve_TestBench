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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class DriveCommands {
	private Drivetrain mDrivetrain;
	private SwerveModule[] mSwerveModules;

	public DriveCommands(Drivetrain drivetrain, SwerveModule[] swerveModules) {
		mDrivetrain = drivetrain;
		mSwerveModules = swerveModules;
	}

	public Command DefaultDriveCommand(CommandJoystick controller) {
		return Commands.run(() -> {
			var strafeX = MathUtil.applyDeadband(controller.getRawAxis(0), 0.1);
			var forwardY = -MathUtil.applyDeadband(controller.getRawAxis(1), 0.1);
			var rotation = controller.getRawAxis(2) - controller.getRawAxis(3);

			mDrivetrain.drive(strafeX, forwardY, rotation, false);
		}, mDrivetrain, mSwerveModules[0], mSwerveModules[1], mSwerveModules[2], mSwerveModules[3]);
	}

	public Command resetGyroComamand() {
		return Commands.runOnce(() -> mDrivetrain.resetGyro(), mDrivetrain);
	}

	public Command followTrajectoryWithEvents(PathPlannerTrajectory trajectory, boolean isFirstPath) {
		return new SequentialCommandGroup(
				new InstantCommand(() -> {
					// Reset odometry for the first path you run during auto
					if (isFirstPath) {
						mDrivetrain.resetOdometry(trajectory.getInitialHolonomicPose());
					}
				}),
				new PPSwerveControllerCommand(
						trajectory,
						mDrivetrain::getPose, // Pose supplier
						new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0
													// will only use feedforwards.
						new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
						new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving
													// them 0 will only use feedforwards.
						mDrivetrain::drive,
						getAllDriveSubsystems()));
	}

	private Subsystem[] getAllDriveSubsystems() {
		return new Subsystem[] { mDrivetrain, mSwerveModules[0], mSwerveModules[1], mSwerveModules[2],
				mSwerveModules[3] };
	}
}
