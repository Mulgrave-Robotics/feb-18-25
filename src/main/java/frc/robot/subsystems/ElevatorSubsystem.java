package frc.robot.subsystems;

// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkFlex leftMotor;
    private final SparkFlex rightMotor;
    private final RelativeEncoder encoder;

    public ElevatorSubsystem() {
        leftMotor = new SparkFlex(ElevatorConstants.kLeftMotorCanId, MotorType.kBrushless);
        rightMotor = new SparkFlex(ElevatorConstants.kRightMotorCanId, MotorType.kBrushless);

        // ✅ Motor Configuration
        SparkFlexConfig globalConfig = new SparkFlexConfig();
        SparkFlexConfig rightMotorConfig = new SparkFlexConfig();

        globalConfig
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kBrake);

        rightMotorConfig
                .apply(globalConfig)
                .inverted(true);

        // ✅ Apply configurations
        leftMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ✅ Encoder
        encoder = leftMotor.getEncoder();
        encoder.setPosition(0);
    }

    private double inchesToRotations(double inches) {
        return (inches - ElevatorConstants.kBaseHeight) * ElevatorConstants.kGearRatio / (2 * Math.PI);
    }

    public void setPosition(double heightInches) {
        double targetRotations = inchesToRotations(heightInches);
        SmartDashboard.putNumber("targetRotations", targetRotations);
        leftMotor.set(targetRotations);
        rightMotor.set(leftMotor.get());
    }

    public void goToLevel1() {
        setPosition(ElevatorConstants.kLevel1Height);
    }

    public void goToLevel2() {
        setPosition(ElevatorConstants.kLevel2Height);
    }

    public void resetToBase() {
        setPosition(ElevatorConstants.kBaseHeight);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}
