// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.*;
import frc.robot.config.DriveMap;
import frc.robot.subsystems.*;

/** Add your docs here. */
public class RobotContainer {
    public Drivetrain Drivetrain;
    private IntakeSubsystem intakeSubsystem;
    private TurretRotation turretRotation;

    // Front Left
    public final SwerveModule FrontLeftSwerveModule = new SwerveModule(
            DriveMap.kFrontLeftDrivingMotorId,
            DriveMap.kFrontLeftSteeringMotorId,
            DriveMap.kFrontLeftEncoderAIOChannel,
            DriveMap.kFrontLeftEncoderOffset);

    // Front Right
    public final SwerveModule FrontRightSwerveModule = new SwerveModule(
            DriveMap.kFrontRightDrivingMotorId,
            DriveMap.kFrontRightSteeringMotorId,
            DriveMap.kFrontRightEncoderAIOChannel,
            DriveMap.kFrontRightEncoderOffset);

    // Rear Left
    public final SwerveModule RearLeftSwerveModule = new SwerveModule(
            DriveMap.kRearLeftDrivingMotorId,
            DriveMap.kRearLeftSteeringMotorId,
            DriveMap.kRearLeftEncoderAIOChannel,
            DriveMap.kRearLeftEncoderOffset);

    // Rear Right
    public final SwerveModule RearRightSwerveModule = new SwerveModule(
            DriveMap.kRearRightDrivingMotorId,
            DriveMap.kRearRightSteeringMotorId,
            DriveMap.kRearRightEncoderAIOChannel,
            DriveMap.kRearRightEncoderOffset);

    private CommandJoystick mController;

    public RobotContainer() {

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

        intakeSubsystem = new IntakeSubsystem();
        intakeSubsystem.register();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Reset gyro Button
        // Reset Gyro
        mController.button(3).onTrue(DriveCommands.resetGyroComamand(Drivetrain));
        // Run Intake buttons
        mController.pov(180)
                .onTrue(Commands.run(() -> {

                    intakeSubsystem.runIntake(0.5 * -1);

                }, intakeSubsystem))
                .onFalse(IntakeCommands.stopIntake(intakeSubsystem));
        mController.pov(0)
                .onTrue(Commands.run(() -> {

                    intakeSubsystem.runIntake(0.5 * 1);

                }, intakeSubsystem))
                .onFalse(IntakeCommands.stopIntake(intakeSubsystem));
        // Move intake button

        // Move turret left
        mController.pov(90)
                .onTrue(TurretCommands.runRotation(turretRotation, 0.2))
                .onFalse(TurretCommands.stop(turretRotation));

        // Move turret right
        mController.pov(270)
                .onTrue(TurretCommands.runRotation(turretRotation, -0.2))
                .onFalse(TurretCommands.stop(turretRotation));

    }
}