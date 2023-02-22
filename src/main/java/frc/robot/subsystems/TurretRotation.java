package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.config.DriveMap;
import frc.robot.prime.utilities.CTREConverter;
import frc.robot.sensors.MA3Encoder;
import prime.movers.LazyWPITalonFX;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretRotation extends SubsystemBase {
    private WPI_TalonFX mRotator;

    public TurretRotation() {
        mRotator = new WPI_TalonFX(22);

    }

    public void rotate(double speed) {
        mRotator.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        mRotator.stopMotor();
    }

}
