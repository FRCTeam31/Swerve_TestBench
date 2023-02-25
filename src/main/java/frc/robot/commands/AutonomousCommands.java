// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/** Add your docs here. */
public class AutonomousCommands {
    public Command runEvents(String key) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("scoreGamePiece", new PrintCommand("Scored A game piece"));

        return eventMap.get(key);

    }

}
