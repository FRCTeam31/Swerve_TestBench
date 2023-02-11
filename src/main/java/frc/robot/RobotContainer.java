package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Commands;
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

        mFrontRightSwerve = new SwerveModule (
            DriveMap.kFrontRightDrivingMotorId, 
            DriveMap.kFrontRightSteeringMotorId, 
            DriveMap.kFrontRightEncoderAIOChannel, 
            DriveMap.kFrontRightEncoderOffset);

        mRearLeftSwerve = new SwerveModule (
            DriveMap.kRearLeftDrivingMotorId, 
            DriveMap.kRearLeftSteeringMotorId, 
            DriveMap.kRearLeftEncoderAIOChannel, 
            DriveMap.kRearLeftEncoderOffset);

        mRearRightSwerve = new SwerveModule (
            DriveMap.kRearRightDrivingMotorId, 
            DriveMap.kRearRightSteeringMotorId, 
            DriveMap.kRearRightEncoderAIOChannel, 
            DriveMap.kRearRightEncoderOffset);
        
        mDrivetrain = new Drivetrain(mFrontLeftSwerve, mFrontRightSwerve, mRearLeftSwerve, mRearRightSwerve);

        configureBindings();
    }

    private void configureBindings() {
        mController = new CommandJoystick(0);

        mController.button(3).onTrue(Commands.runOnce(() -> {
            mDrivetrain.resetGyro();
            System.out.println("[DRIVE] Reset gyro");
        }));
    }
}
