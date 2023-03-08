// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.*;
import frc.robot.config.DriveMap;
import frc.robot.subsystems.*;

/** Add your docs here. */
public class RobotContainer {

    // Subsystems
    public Drivetrain Drivetrain;
    private IntakeSubsystem intakeSubsystem;
    private Flywheel Flywheel;
    private TurretRotation turretRotation;

    // Autonomous Paths

    // Front Left
    public final SwerveModule FrontLeftSwerveModule = new SwerveModule(
            "FrontLeft",
            DriveMap.kFrontLeftDrivingMotorId,
            DriveMap.kFrontLeftSteeringMotorId,
            DriveMap.kFrontLeftEncoderAIOChannel,
            DriveMap.kFrontLeftEncoderOffset,
            false);

    // Front Right
    public final SwerveModule FrontRightSwerveModule = new SwerveModule(
            "FrontRight",
            DriveMap.kFrontRightDrivingMotorId,
            DriveMap.kFrontRightSteeringMotorId,
            DriveMap.kFrontRightEncoderAIOChannel,
            DriveMap.kFrontRightEncoderOffset,
            false);

    // Rear Left
    public final SwerveModule RearLeftSwerveModule = new SwerveModule(
            "RearLeft",
            DriveMap.kRearLeftDrivingMotorId,
            DriveMap.kRearLeftSteeringMotorId,
            DriveMap.kRearLeftEncoderAIOChannel,
            DriveMap.kRearLeftEncoderOffset,
            false);

    // Rear Right
    public final SwerveModule RearRightSwerveModule = new SwerveModule(
            "RearRight",
            DriveMap.kRearRightDrivingMotorId,
            DriveMap.kRearRightSteeringMotorId,
            DriveMap.kRearRightEncoderAIOChannel,
            DriveMap.kRearRightEncoderOffset,
            false);

    private CommandJoystick mController;

    public RobotContainer() {

        intakeSubsystem = new IntakeSubsystem();

        SwerveModule[] modules = new SwerveModule[4];
        modules[0] = FrontLeftSwerveModule;
        modules[1] = FrontRightSwerveModule;
        modules[2] = RearLeftSwerveModule;
        modules[3] = RearRightSwerveModule;

        turretRotation = new TurretRotation();

        mController = new CommandJoystick(0);
        Drivetrain = new Drivetrain(FrontLeftSwerveModule, FrontRightSwerveModule, RearLeftSwerveModule,
                RearRightSwerveModule);
        Drivetrain.setDefaultCommand(DriveCommands.DefaultDriveCommand(mController, Drivetrain, modules));
        Drivetrain.register();

        Flywheel = new Flywheel();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Reset drivetrain gyro heading
        mController.button(3) // Y button
                .onTrue(DriveCommands.resetGyroComamand(Drivetrain));

        // Shift drive speed
        mController.button(1) // A button
                .onTrue(DriveCommands.shiftDriveSpeedCommand(Drivetrain));

        // Run flywheel at 75% speed when Start is pressed, and turn it off when it's
        // pressed again
        mController.button(8) // Start button
                .onTrue(Commands.runOnce(() -> {
                    if (Flywheel.getEnabled()) {
                        Flywheel.setSpeed(0);
                        Flywheel.setEnabled(false);
                    } else {
                        Flywheel.setSpeed(Flywheel.kMaxRpm * 0.75);
                        Flywheel.setEnabled(true);
                    }
                }, Flywheel));
        // Run intake in and out
        // Left and right pov
        mController.pov(0)
                .onTrue(IntakeCommands.runIntake(intakeSubsystem, 1))
                .onFalse(IntakeCommands.stopIntake(intakeSubsystem));

        mController.pov(180)
                .onTrue(IntakeCommands.runIntake(intakeSubsystem, -1))
                .onFalse(IntakeCommands.stopIntake(intakeSubsystem));

        // Move intake in and out
        // Bumpers
        mController.button(5).onTrue(IntakeCommands.setIntakeCommand(intakeSubsystem, true));
        mController.button(6).onTrue(IntakeCommands.setIntakeCommand(intakeSubsystem, false));

        // Move turret left
        mController.pov(90)
                .onTrue(TurretCommands.runRotation(turretRotation, 0.2))
                .onFalse(TurretCommands.stop(turretRotation));

        // Move turret right
        mController.pov(270)
                .onTrue(TurretCommands.runRotation(turretRotation, -0.2))
                .onFalse(TurretCommands.stop(turretRotation));
    }

    public Command getAutonomousCommand() {

        PathPlannerTrajectory driveForwardOneMeter = PathPlanner.loadPath("DriveForwardOneMeter",
                new PathConstraints(0.1, 0.01));
        return DriveCommands.followTrajectoryWithEvent(Drivetrain, driveForwardOneMeter, true);

    }

}