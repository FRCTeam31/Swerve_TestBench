// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommands;
import frc.robot.config.DriveMap;
import frc.robot.subsystems.*;

/** Add your docs here. */
public class RobotContainer {
        private Drivetrain Drivetrain;
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

                Flywheel = new Flywheel();

                configureButtonBindings();
        }

        private void configureButtonBindings() {
                mController.button(3).onTrue(DriveCommands.resetGyroComamand(Drivetrain));

                // Run flywheel at max while the button is held
                mController.button(6)
                                .onTrue(Commands.runOnce(() -> Flywheel.setSpeed(Flywheel.kMaxRpm), Flywheel))
                                .onFalse(Commands.runOnce(() -> Flywheel.setSpeed(0), Flywheel));
        }
}
