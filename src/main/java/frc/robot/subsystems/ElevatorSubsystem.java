package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkFlex upperMotor;
    private final SparkFlex lowerMotor;
    private final RelativeEncoder encoder;
    private int currentLevel;

    public ElevatorSubsystem() {
        // ✅ Ensure CAN IDs match actual hardware
        upperMotor = new SparkFlex(ElevatorConstants.elevatorUpperMotorID, MotorType.kBrushless);
        lowerMotor = new SparkFlex(ElevatorConstants.elevatorLowerMotorID, MotorType.kBrushless);

        // ✅ Motor Configuration
        SparkFlexConfig globalConfig = new SparkFlexConfig();
        SparkFlexConfig lowerMotorConfig = new SparkFlexConfig();

        globalConfig
                .smartCurrentLimit(40)
                .idleMode(IdleMode.kBrake);

        lowerMotorConfig
                .apply(globalConfig)
                .inverted(true); // Make sure motors move in sync

        // ✅ Apply configurations
        upperMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lowerMotor.configure(lowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ✅ Encoder setup
        encoder = upperMotor.getEncoder();
        encoder.setPosition(0);
        currentLevel = 0;
    }

    public double getPositionInches() {
        return encoder.getPosition() * ElevatorConstants.kPositionConversionFactor;
    }

    public void moveToHeight(double targetHeight) {
        double currentHeight = getPositionInches();
        double speed = (targetHeight > currentHeight) ? ElevatorConstants.kMaxSpeedPercentage : -ElevatorConstants.kMaxSpeedPercentage;

        upperMotor.set(speed);
        lowerMotor.set(speed);

        SmartDashboard.putNumber("Elevator Target Height", targetHeight);
        SmartDashboard.putNumber("Elevator Speed", speed);
        SmartDashboard.putNumber("Current Elevator Height", currentHeight);
    }

    public Command goToLevel(int level) {
        double targetHeight;
        switch (level) {
            case 1: targetHeight = ElevatorConstants.vL1Height; break;
            case 2: targetHeight = ElevatorConstants.vL2Height; break;
            case 3: targetHeight = ElevatorConstants.vL3Height; break;
            case 4: targetHeight = ElevatorConstants.vL4Height; break;
            default: targetHeight = ElevatorConstants.vL1Height;
        }

        return run(() -> moveToHeight(targetHeight)).until(() -> Math.abs(getPositionInches() - targetHeight) < ElevatorConstants.kElevatorDefaultTolerance)
                .andThen(() -> {
                    upperMotor.set(0);
                    lowerMotor.set(0);
                    currentLevel = level;
                });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPositionInches());
        SmartDashboard.putNumber("Elevator Level", currentLevel);
    }
}
