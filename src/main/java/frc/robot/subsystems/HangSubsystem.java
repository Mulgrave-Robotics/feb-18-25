package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangConstants;

public class HangSubsystem extends SubsystemBase {
    private final SparkMax hangMotor;

    public HangSubsystem() {
        // ✅ Initialize motor with correct CAN ID
        hangMotor = new SparkMax(HangConstants.HangMotorID, MotorType.kBrushless);

        // ✅ Motor Configuration
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig
                .smartCurrentLimit(50)  // Set current limit to 50A
                .idleMode(IdleMode.kBrake); // Set motor to brake mode

        // ✅ Apply configurations
        hangMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ✅ Move hang down (clamp)
    public Command hangDown() {
        return run(() -> {
            hangMotor.set(1.0);
        });
    }

    // ✅ Move hang up (release)
    public Command hangUp() {
        return run(() -> {
            hangMotor.set(-1.0); 
          
        });
    }

    // ✅ Stop the motor
    public Command stop() {
        return run(() -> {
            hangMotor.set(0);
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hang Motor Output", hangMotor.get());
    }
}
