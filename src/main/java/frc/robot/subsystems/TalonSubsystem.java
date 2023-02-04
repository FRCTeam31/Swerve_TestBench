package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotMap;

public class TalonSubsystem extends SubsystemBase {
    private TalonSRX talon1;

    /** Creates a new ExampleSubsystem. */
    public TalonSubsystem() {
        talon1 = new WPI_TalonSRX(RobotMap.TALON1_CAN_ID);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // This is generally used to update/poll sensors and actuators controlled by them
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    public void set(double val) {
        talon1.set(ControlMode.PercentOutput, val);
    }
}
