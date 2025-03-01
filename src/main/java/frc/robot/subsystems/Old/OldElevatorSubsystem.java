package frc.robot.subsystems.Old;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class OldElevatorSubsystem extends SubsystemBase {
    private final SparkFlex upperMotor;
    private final SparkFlex lowerMotor;
    private final RelativeEncoder encoder;
    private int currentLevel;

    public OldElevatorSubsystem() {
        // ✅ Initialize motors with correct CAN IDs
        upperMotor = new SparkFlex(ElevatorConstants.elevatorUpperMotorID, MotorType.kBrushless);
        lowerMotor = new SparkFlex(ElevatorConstants.elevatorLowerMotorID, MotorType.kBrushless);

        // ✅ Configure motors
        SparkFlexConfig globalConfig = new SparkFlexConfig();
        SparkFlexConfig lowerMotorConfig = new SparkFlexConfig();

        globalConfig
                .smartCurrentLimit(40) // Current limit
                .idleMode(IdleMode.kBrake); // Brake when not moving

        lowerMotorConfig
                .apply(globalConfig)
                .inverted(true); // Invert lower motor to sync movement

        // ✅ Apply configurations
        upperMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lowerMotor.configure(lowerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ✅ Encoder setup
        encoder = upperMotor.getEncoder();
        encoder.setPosition(0);
        currentLevel = 0;
    }

    // ✅ Convert encoder position to inches
    public double getPositionInches() {
        return encoder.getPosition() * ElevatorConstants.kPositionConversionFactor;
    }

    // ✅ Move elevator to a specified height
    public void moveToHeight(double targetHeight) {
        double currentHeight = getPositionInches();
        double speed = (targetHeight > currentHeight) ? ElevatorConstants.kMaxSpeedPercentage
                : -ElevatorConstants.kMaxSpeedPercentage;

        upperMotor.set(speed);
        lowerMotor.set(speed);

        SmartDashboard.putNumber("Elevator Target Height", targetHeight);
        SmartDashboard.putNumber("Elevator Speed", speed);
        SmartDashboard.putNumber("Current Elevator Height", currentHeight);
    }

    // ✅ Command to move the elevator to a specific level
    public Command goToLevel(int level) {
        double targetHeight;
        switch (level) {
            case 1:
                targetHeight = ElevatorConstants.vL1Height;
                break;
            case 2:
                targetHeight = ElevatorConstants.vL2Height;
                break;
            case 3:
                targetHeight = ElevatorConstants.vL3Height;
                break;
            case 4:
                targetHeight = ElevatorConstants.vL4Height;
                break;
            default:
                targetHeight = ElevatorConstants.vL1Height; // Default to base level
        }

        return run(() -> moveToHeight(targetHeight))
                .until(() -> MathUtil.isNear(getPositionInches(), targetHeight,
                        ElevatorConstants.kElevatorDefaultTolerance))
                .andThen(() -> {
                    upperMotor.set(0);
                    lowerMotor.set(0);
                    currentLevel = level;
                });
    }

    public Command moveUp() {
        int targetLevel = Math.min(currentLevel + 1, ElevatorConstants.kMaxLevel);
        return goToLevel(targetLevel);
    }

    public Command moveDown() {
        int targetLevel = Math.max(currentLevel - 1, ElevatorConstants.kMinLevel);
        return goToLevel(targetLevel);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position", getPositionInches());
        SmartDashboard.putNumber("Elevator Level", currentLevel);
    }
}
