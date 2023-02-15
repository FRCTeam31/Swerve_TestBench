// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.driveCommand;

/** Add your docs here. */
public class RobotContainer {
    private SwerveDriveTrainSubsystem mSwerve;
    private driveCommand drive;
    private CommandJoystick mController;

    
    


    public RobotContainer(){
        mSwerve = new SwerveDriveTrainSubsystem();
        mController = new CommandJoystick(0);
        drive = new driveCommand(false, mSwerve);
        
    

        

    }

 



 

}
