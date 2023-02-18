// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/** Add your docs here. */
public class RobotContainer {
        private Drivetrain Drivetrain;

        private CommandJoystick mController;

        public RobotContainer() {
                mController = new CommandJoystick(0);
                Drivetrain = new Drivetrain();
                Drivetrain.setDefaultCommand(Commands.run(() -> {
                        var strafeX = MathUtil.applyDeadband(mController.getRawAxis(0), 0.1);
                        var forwardY = mController.getRawAxis(1);
                        var rotation = mController.getRawAxis(2) + mController.getRawAxis(3);

                        Drivetrain.drive(strafeX, forwardY, rotation, false);
                }, Drivetrain));
                Drivetrain.register();
        }

}
