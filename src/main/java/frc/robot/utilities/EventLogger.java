package frc.robot.utilities;

public class EventLogger {
    public static void Information(String subsystemName, String message) {
        System.out.println("[" + subsystemName.toUpperCase() + "] " + message);
    }
}
