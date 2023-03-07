// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.IntakeMap;

public class IntakeSubsystem extends SubsystemBase {
    // private WPI_TalonSRX intakeMotor;
    // private WPI_TalonSRX elevatorIntakeMotor;

    // private Compressor compressor;
    // private DoubleSolenoid intakeArmSolenoid;

    public IntakeSubsystem() {
        // Set up Motors
        // intakeMotor = new WPI_TalonSRX(IntakeMap.kIntakeMotorId);
        // intakeMotor.setInverted(true);
        // elevatorIntakeMotor = new WPI_TalonSRX(IntakeMap.kElevatorIntakeMotor);

        // compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        // compressor.enableDigital();

        // intakeArmSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    }

    public void runIntake(double speed) {
        // intakeMotor.set(ControlMode.PercentOutput, speed);
        // elevatorIntakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopIntake() {
        // intakeMotor.stopMotor();
        // elevatorIntakeMotor.stopMotor();
    }

    public void setIntakeOut(boolean out) {
        // intakeArmSolenoid.set(out ? Value.kForward : Value.kReverse);
    }
}
