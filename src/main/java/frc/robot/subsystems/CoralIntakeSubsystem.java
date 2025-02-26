package frc.robot.subsystems;

// import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class CoralIntakeSubsystem extends SubsystemBase {
    private final SparkMax m_rollerMoter;
    // private final RelativeEncoder m_encoder; // do we need it?

    public CoralIntakeSubsystem() {
        // update the canID in Constants.java
        m_rollerMoter = new SparkMax(11, MotorType.kBrushless);

        // ✅ Motor Configuration
        SparkMaxConfig moterConfig = new SparkMaxConfig();

        moterConfig
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kBrake);

        // ✅ Apply configurations
        m_rollerMoter.configure(moterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ✅ Encoder
        //m_encoder = m_rollerMotor.getEncoder();
        //encoder.setPosition(0);
    }
    
    public Command spitCoralOut(){
        return run(()-> m_rollerMoter.set(0));

    }
  
}
