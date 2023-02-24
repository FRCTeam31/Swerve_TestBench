// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.naming.NotContextException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.config.DriveMap;
import frc.robot.subsystems.*;

/** Add your docs here. */
public class RobotContainer {
    private Drivetrain Drivetrain;
    private IntakeSubsystem intakeSubsystem;
    private Flywheel Flywheel;

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

        mController = new CommandJoystick(0);
        Drivetrain = new Drivetrain(FrontLeftSwerveModule, FrontRightSwerveModule, RearLeftSwerveModule,
                RearRightSwerveModule);
        Drivetrain.setDefaultCommand(DriveCommands.DefaultDriveCommand(mController, Drivetrain, modules));
        Drivetrain.register();

        intakeSubsystem = new IntakeSubsystem();
        intakeSubsystem.setDefaultCommand(IntakeCommands.runIntake(intakeSubsystem, mController));
        intakeSubsystem.register();

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

        // Move intake in and out
        // Bumpers
        mController.button(5).onTrue(IntakeCommands.setIntakeCommand(intakeSubsystem, true));
        mController.button(6).onTrue(IntakeCommands.setIntakeCommand(intakeSubsystem, false));
    }
}