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
import frc.robot.commands.*;
import frc.robot.config.DriveMap;
import frc.robot.subsystems.*;

/** Add your docs here. */
public class RobotContainer {
    public Drivetrain Drivetrain;
    private IntakeSubsystem intakeSubsystem;
    private Compressor compressor;

    public Solenoid leftSolenoid;
    public Solenoid rightSolenoid;
    private TurretRotation turretRotation;
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

        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
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
                intakeSubsystem = new IntakeSubsystem();
        intakeSubsystem.register();

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Reset gyro Button
        mController.button(3).onTrue(DriveCommands.resetGyroComamand(Drivetrain));

        // Run flywheel at 75% speed when "Start" is pressed, and turn it off when it's
        // pressed again
        mController.button(8)
                .onTrue(Commands.runOnce(() -> {
                    if (Flywheel.getEnabled()) {
                        Flywheel.setSpeed(0);
                        Flywheel.setEnabled(false);
                    } else {
                        Flywheel.setSpeed(Flywheel.kMaxRpm * 0.75);
                        Flywheel.setEnabled(true);
                    }
                }, Flywheel));
    }
}