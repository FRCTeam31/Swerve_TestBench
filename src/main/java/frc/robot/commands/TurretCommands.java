package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.TurretRotation;

public class TurretCommands {
    public static Command runRotation(TurretRotation turret, double speed) {
        return Commands.runOnce(() -> {
            turret.rotate(speed);
        });
    }

}
