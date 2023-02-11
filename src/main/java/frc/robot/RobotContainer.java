package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.config.DriveMap;

public class RobotContainer {
    public CommandJoystick mController;
    public SwerveModule mFrontLeftSwerve;
    public SwerveModule mFrontRightSwerve;
    public SwerveModule mRearLeftSwerve;
    public SwerveModule mRearRightSwerve;
    public Drivetrain mDrivetrain;

    public RobotContainer() {
        mFrontLeftSwerve = new SwerveModule(
            DriveMap.kFrontLeftDrivingMotorId, 
            DriveMap.kFrontLeftSteeringMotorId, 
            DriveMap.kFrontLeftEncoderAIOChannel,
            DriveMap.kFrontLeftEncoderOffset);

        mFrontRightSwerve = new SwerveModule(
            DriveMap.kFrontRightDrivingMotorId, 
            DriveMap.kFrontRightSteeringMotorId, 
            DriveMap.kFrontRightEncoderAIOChannel, 
            DriveMap.kFrontRightEncoderOffset);

        mRearLeftSwerve = new SwerveModule(
            DriveMap.kRearLeftDrivingMotorId, 
            DriveMap.kRearLeftSteeringMotorId, 
            DriveMap.kRearLeftEncoderAIOChannel, 
            DriveMap.kRearLeftEncoderOffset);

        mRearRightSwerve = new SwerveModule(
            DriveMap.kRearRightDrivingMotorId, 
            DriveMap.kRearRightSteeringMotorId, 
            DriveMap.kRearRightEncoderAIOChannel, 
            DriveMap.kRearRightEncoderOffset);
        
        mDrivetrain = new Drivetrain(mFrontLeftSwerve, mFrontRightSwerve, mRearLeftSwerve, mRearRightSwerve);

        configureBindings();
    }

    private void configureBindings() {
        mController = new CommandJoystick(0);

        // Default commands
        mDrivetrain.setDefaultCommand(Commands.run(() -> {
            // Grab the X and Y axis from the left joystick on the controller
            var strafeX = mController.getRawAxis(0);
            var forwardY = -mController.getRawAxis(1);

            // Right trigger should rotate the robot clockwise, left counterclockwise
            // Add the two [0,1] trigger axes together for a combined period of [-1, 1]
            var rotation = mController.getRawAxis(2) + -mController.getRawAxis(3);

            mDrivetrain.drive(strafeX, forwardY, rotation, true);
        }, mDrivetrain, mFrontLeftSwerve, mFrontRightSwerve, mRearLeftSwerve, mRearRightSwerve));

        // Button bindings
        mController.button(3).onTrue(Commands.runOnce(() -> {
            mDrivetrain.resetGyro();
            System.out.println("[DRIVE] Reset gyro");
        }));
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
