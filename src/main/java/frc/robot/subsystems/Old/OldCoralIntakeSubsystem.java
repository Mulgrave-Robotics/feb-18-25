package frc.robot.subsystems.Old;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class OldCoralIntakeSubsystem extends SubsystemBase {
    private final SparkMax m_rollerMotor;

    public OldCoralIntakeSubsystem() {
        // Use the constant from IntakeConstants
        m_rollerMotor = new SparkMax(IntakeConstants.coralMotorID, MotorType.kBrushless);

        // Motor Configuration
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake);

        // Apply configurations
        m_rollerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command intakeCoral() {
        return run(() -> m_rollerMotor.set(IntakeConstants.CoralIntakeSpeeds));
    }

    public Command outakeCoral() {
        return run(() -> m_rollerMotor.set(IntakeConstants.CoralOuttakeSpeeds));
    }

    public Command stopIntake() {
        return runOnce(() -> m_rollerMotor.set(0));
    }

    @Override
    public void periodic() {
        // Add motor output to SmartDashboard for debugging
        SmartDashboard.putNumber("coral_intake_speed", m_rollerMotor.get());
    }
}