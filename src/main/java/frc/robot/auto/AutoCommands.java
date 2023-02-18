package frc.robot.auto;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class AutoCommands {
    public static HashMap<String, Command> AutoEvents = new HashMap<>(Map.ofEntries(
            Map.entry("RaiseArm", Commands.run(() -> System.out.println("====[AUTO] Event - RaiseArm"))),
            Map.entry("LowerArm", Commands.run(() -> System.out.println("====[AUTO] Event - LowerArm"))),
            Map.entry("ScoreCube", Commands.run(() -> System.out.println("====[AUTO] Event - ScoreCube"))),
            Map.entry("BalanceOnPlatform",
                    Commands.run(() -> System.out.println("====[AUTO] Event - BalanceOnPlatform")))));

    public static Command RunAutoPath(String pathName) {
        var trajectory = PathPlanner.loadPath(pathName, new PathConstraints(2, 1));

        var swerveControllerCommand = new 
    }
}
