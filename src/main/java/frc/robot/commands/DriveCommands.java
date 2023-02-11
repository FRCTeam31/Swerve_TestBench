package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Drivetrain;

public class DriveCommands {
    public static Command DriveWithJoystick(CommandJoystick js, Drivetrain drive, boolean fieldRelative)
    {
        return Commands.run(() -> {
            // Grab the X and Y axis from the left joystick on the controller
            var strafeX = js.getRawAxis(0);
            var forwardY = -js.getRawAxis(1);
      
            // Right trigger should rotate the robot clockwise, left counterclockwise
            // Add the two [0,1] trigger axes together for a combined period of [-1, 1]
            var rotation = js.getRawAxis(2) + -js.getRawAxis(3);
      
            drive.drive(strafeX, forwardY, rotation, fieldRelative);
          }, drive);
    }

    public static Command ResetGyro(Drivetrain drive) {
        return Commands.runOnce(() -> drive.resetGyro());
    }
}
