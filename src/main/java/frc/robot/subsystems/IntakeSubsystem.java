// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.IntakeMap;

public class IntakeSubsystem extends SubsystemBase {

  private WPI_TalonSRX intakeMotor;
  private WPI_TalonSRX elevatorIntakeMotor;

  private Compressor compressor;
  private Solenoid leftSolenoid;
  private Solenoid rightSolenoid;

  /** Creates a new intakeSubsystem. */

  public IntakeSubsystem() {
    // Set up Motors
    intakeMotor = new WPI_TalonSRX(IntakeMap.kIntakeMotorId);
    elevatorIntakeMotor = new WPI_TalonSRX(IntakeMap.kElevatorIntakeMotor);
    elevatorIntakeMotor.setInverted(true);

    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();

    leftSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    rightSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

    // Set up pneumatics
  }

  public void runIntake(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
    elevatorIntakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
    elevatorIntakeMotor.stopMotor();
  }

  public void moveIntake(boolean direction) {
    leftSolenoid.set(direction);
    rightSolenoid.set(direction);
  }

}
