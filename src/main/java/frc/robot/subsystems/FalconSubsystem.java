// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.RobotMap;

public class FalconSubsystem extends SubsystemBase {
  private WPI_TalonFX falcon1;
  /** Creates a new falconSubsystem. */
  public FalconSubsystem() {
    falcon1 = new WPI_TalonFX(RobotMap.FALCON1_CAN_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void set(double val){
    falcon1.set(val);
  }
}
