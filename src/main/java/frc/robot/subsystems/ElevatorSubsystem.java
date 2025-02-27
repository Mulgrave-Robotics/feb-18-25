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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkFlex leftMotor;
    private final SparkFlex rightMotor;
    private final RelativeEncoder encoder;
    private int currentLevel;

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
        currentLevel = 0;
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

    //Lok's approach

    public double getPositionInches() {
        return encoder.getPosition() * (2 * Math.PI * ElevatorConstants.kGearRatio);           
    }

    public void reachLevel(int targetLevel, int direction){
        
        if (direction > 0) {
            // move up
            if (targetLevel <= ElevatorConstants.kMaxLevel) {
                leftMotor.set(ElevatorConstants.kMaxSpeedPercentage*direction);
                currentLevel++;
                SmartDashboard.putNumber("currentLevel", currentLevel);
            }
        } else {
            // move down
            if (targetLevel >= ElevatorConstants.kMinLevel) {
                leftMotor.set(ElevatorConstants.kMaxSpeedPercentage*direction);
                currentLevel--;
                SmartDashboard.putNumber("currentLevel", currentLevel);
            }

        }
        
    }

    public Command setLevel(int targetLevel, int direction){
        return run(()-> reachLevel(targetLevel, direction));
    }
    
    // Move one level up
    public Command moveUp(){
        double targetHeight;
        switch(currentLevel) {
            case 0:
                // move to level 1
                targetHeight = ElevatorConstants.kLevel1Height - ElevatorConstants.kBaseHeight;
                break;
            default:
                // move to level 2 (max level)
                targetHeight = ElevatorConstants.kLevel2Height - ElevatorConstants.kBaseHeight;
        }
        SmartDashboard.putNumber("targetHeight", targetHeight);
        SmartDashboard.putNumber("targetLevel", currentLevel+1);
        return setLevel(currentLevel+1,1).until(()->aroundHeight(targetHeight)).andThen(() -> leftmotor.set(0.0));
    }

    // Move one level down
    public Command moveDown(){
        double targetHeight;
        switch(currentLevel) {
            case 2:
                // move to level 1
                targetHeight = ElevatorConstants.kLevel1Height - ElevatorConstants.kBaseHeight;
                break;
            default:
                // return to base
                targetHeight = 0;
        }
        SmartDashboard.putNumber("targetHeight", targetHeight);
        SmartDashboard.putNumber("targetLevel", currentLevel-1);
        return setLevel(currentLevel-1,-1).until(()->aroundHeight(targetHeight)).andThen(() -> leftmotor.set(0.0));
    }

    public boolean aroundHeight(double height){
        return aroundHeight(height, ElevatorConstants.kElevatorDefaultTolerance);
    }
    public boolean aroundHeight(double height, double tolerance){
        return MathUtil.isNear(height,getPositionInches(),tolerance);
    }
}
