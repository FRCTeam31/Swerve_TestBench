package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.DriveCommands;
import frc.robot.config.*;

public class RobotContainer {
    public CommandJoystick mController;
    public SwerveModule mFrontLeftSwerve;
    public SwerveModule mFrontRightSwerve;
    public SwerveModule mRearLeftSwerve;
    public SwerveModule mRearRightSwerve;
    public Drivetrain mDrivetrain;

    public RobotContainer() {
        mFrontLeftSwerve = new SwerveModule(
            CANMap.kFrontLeftDrivingMotorId, 
            CANMap.kFrontLeftSteeringMotorId, 
            DriveMap.kFrontLeftEncoderAIOChannel,
            DriveMap.kFrontLeftEncoderOffset);

        mFrontRightSwerve = new SwerveModule(
            CANMap.kFrontRightDrivingMotorId, 
            CANMap.kFrontRightSteeringMotorId, 
            DriveMap.kFrontRightEncoderAIOChannel, 
            DriveMap.kFrontRightEncoderOffset);

        mRearLeftSwerve = new SwerveModule(
            CANMap.kRearLeftDrivingMotorId, 
            CANMap.kRearLeftSteeringMotorId, 
            DriveMap.kRearLeftEncoderAIOChannel, 
            DriveMap.kRearLeftEncoderOffset);

        mRearRightSwerve = new SwerveModule(
            CANMap.kRearRightDrivingMotorId, 
            CANMap.kRearRightSteeringMotorId, 
            DriveMap.kRearRightEncoderAIOChannel, 
            DriveMap.kRearRightEncoderOffset);
        
        mDrivetrain = new Drivetrain(mFrontLeftSwerve, mFrontRightSwerve, mRearLeftSwerve, mRearRightSwerve);

        configureBindings();
    }

    private void configureBindings() {
        mController = new CommandJoystick(0);

        // Subsystem default commands
        mDrivetrain.setDefaultCommand(DriveCommands.DriveWithJoystick(mController, mDrivetrain, true));

        // Button bindings
        mController.button(3).onTrue(DriveCommands.ResetGyro(mDrivetrain));
    }

    public Command getAutonomousCommand() {
        return new InstantCommand(); // TODO: Create the auto programs and select one here
    }
}
