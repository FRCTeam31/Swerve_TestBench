package frc.robot.auto;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;

public final class AutoCommands {
    public static HashMap<String, Command> EventMap = new HashMap<>(Map.ofEntries(
            Map.entry("RaiseArm", Commands.run(() -> System.out.println("====[AUTO] Event - RaiseArm"))),
            Map.entry("LowerArm", Commands.run(() -> System.out.println("====[AUTO] Event - LowerArm"))),
            Map.entry("ScoreCube", Commands.run(() -> System.out.println("====[AUTO] Event - ScoreCube"))),
            Map.entry("BalanceOnPlatform",
                    Commands.run(() -> System.out.println("====[AUTO] Event - BalanceOnPlatform")))));

    public static PathPlannerTrajectory getAutoPath(String pathName) {
        return PathPlanner.loadPath(pathName, new PathConstraints(2, 1));
    }

    public static Command runAutoPath(String pathName, DriveCommands driveCommands) {
        var path = getAutoPath(pathName);

        return new FollowPathWithEvents(driveCommands.followTrajectoryWithEvents(path, true), path.getMarkers(),
                EventMap);
    }
}
