package frc.robot.subsystems;

// import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkFlex upperMotor;
    private final SparkFlex lowerMotor;
    private final RelativeEncoder encoder;
    private double currentHeight;

    public ElevatorSubsystem() {
        upperMotor = new SparkFlex(ElevatorConstants.elevatorUpperMotorID, MotorType.kBrushless);
        lowerMotor = new SparkFlex(ElevatorConstants.elevatorLowerMotorID, MotorType.kBrushless);

        currentHeight = 0;

        // Motor Configuration
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig
                .smartCurrentLimit(40) // Set current limit to 40A
                .idleMode(IdleMode.kBrake);

        // ✅ Apply configurations
        upperMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lowerMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ✅ Encoder setup
        encoder = upperMotor.getEncoder();
        // get positive direction motor encoder
        // positive one should be the motor that is not sucky
        encoder.setPosition(0);
    }

    public double getPositionInches() {
        // 0.9848 is sin(79.99), converts slanted height into vertical height
        return (0.9848 * encoder.getPosition() * (2 * Math.PI * ElevatorConstants.kGearRatio));
    }

    public void reachLevel(double targetHeight) {

        currentHeight = getPositionInches();
        double speed = 0;

        if (targetHeight > currentHeight) {

            speed = ElevatorConstants.kMaxSpeedPercentage * 1.0;

        }

        else if (targetHeight < currentHeight) {
            speed = ElevatorConstants.kMaxSpeedPercentage * -1.0;

        }

        else {
            SmartDashboard.putString("elevator height status", "Elevator is already at wanted height!");
        }

        upperMotor.set(speed);
        lowerMotor.set(speed); // Sync lower motor with upper motor
        SmartDashboard.putNumber("speed", speed);
        SmartDashboard.putNumber("currentHeight", currentHeight);

    }

    public Command setLevel(double targetHeight) {
        return run(() -> reachLevel(targetHeight));
    }

    public Command moveTo(double targetHeight) {
        SmartDashboard.putNumber("targetHeight", targetHeight);

        return setLevel(targetHeight)
                .until(() -> aroundHeight(targetHeight))
                .andThen(() -> {
                    upperMotor.set(0.0);
                    lowerMotor.set(0.0);
                });

    }

    public boolean aroundHeight(double height) {
        return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    }

    public boolean aroundHeight(double height, double tolerance) {
        return MathUtil.isNear(height, getPositionInches(), tolerance);
    }
}
