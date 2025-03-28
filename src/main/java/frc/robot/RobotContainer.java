package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HangSubsystem;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();
    private final AlgaeSubsystem AlgaeIntake = new AlgaeSubsystem();
    private final HangSubsystem m_hang = new HangSubsystem(); 

    // Controllers
    private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    // private final CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kSecondaryControllerPort);

    // Speed multipliers
    private double speedMultiplier = 0.5;

    private boolean lastCameraSwitchState = false; // False is for video 1

    public RobotContainer() {
        configureButtonBindings();

        // Default Driving Control (Main Controller)
        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * speedMultiplier,
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * speedMultiplier,
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) * speedMultiplier,
                    true
                ),
                m_robotDrive
            )
        );
    }

    // public boolean shouldSwitchCameras() {
        // boolean current = m_operatorController.button(ButtonConstants.xboxA).getAsBoolean();
        // boolean shouldSwitch = current && !lastCameraSwitchState;
        // lastCameraSwitchState = current;
        // return shouldSwitch;
    //}       
    
    public Command AlgaeMacro() {
        return new SequentialCommandGroup(
            
            new RunCommand(() -> m_robotDrive.drive(-0.1, 0, 0, false), m_robotDrive)
                .withTimeout(0.3),
            new InstantCommand(() -> m_robotDrive.drive(0.0, 0, 0, false)), // Stop movement
            AlgaeIntake.setAlgaeRoller(IntakeConstants.AlgaeIntakeSpeed),
            new InstantCommand(() -> m_elevator.moveTo(20.5)),
            new RunCommand(() -> m_robotDrive.drive(0.1, 0, 0, false), m_robotDrive)
                .withTimeout(0.3),
            new InstantCommand(() -> m_robotDrive.drive(0.0, 0, 0, false))// Stop movement
            
        );
    }

    private void configureButtonBindings() {

        // MAIN KEYBINDS
        m_driverController.rightTrigger()
            .whileTrue(coralIntake.setCoralIntakeRoller(0))
            .onFalse(new InstantCommand(() -> AlgaeIntake.setAlgaeRoller(0))); // Stops when released

        m_driverController.leftTrigger()
            .whileTrue(coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralOuttakeSpeeds))
            .onFalse(new InstantCommand(() -> coralIntake.setCoralIntakeRoller(0))); // Stops when released

        // m_driverController.leftBumper()
        //     .whileTrue(coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralIntakeSpeeds * 0.5))
        //     .onFalse(new InstantCommand(() -> coralIntake.setCoralIntakeRoller(0))); // Stops when released

        m_driverController.button(ButtonConstants.xboxLB).onTrue(AlgaeIntake.setAlgaeRoller(Constants.IntakeConstants.AlgaeIntakeSpeed));
        m_driverController.button(ButtonConstants.xboxRB).onTrue(AlgaeIntake.setAlgaeRoller(0));
        m_driverController.button(ButtonConstants.xboxX).onTrue(coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralIntakeSpeeds * 0.5));
        m_driverController.button(ButtonConstants.xboxLINES).onTrue(m_elevator.moveTo(20.5));



        
        // m_driverController.rightBumper()
        // .onTrue(AlgaeIntake.setAlgaeRoller(Constants.IntakeConstants.CoralIntakeSpeeds * 0.0))
        //     .onFalse(new InstantCommand(() -> AlgaeIntake.setAlgaeRoller(0)))
        // ;
        // m_driverController.button(ButtonConstants.xboxY).onTrue(coralIntake.setCoralIntakeRoller(0));

        // OPPERATOR KEYBINDS
                
        // Elevator & Hang
        m_driverController.button(ButtonConstants.xboxY).onTrue(m_elevator.moveTo(ElevatorConstants.vL3Height));
        m_driverController.button(ButtonConstants.xboxB).onTrue(m_elevator.moveTo(ElevatorConstants.vL2Height));
        m_driverController.button(ButtonConstants.xboxA).onTrue(m_elevator.moveTo(ElevatorConstants.vL1Height));

        // m_driverController.button(ButtonConstants.xboxY).whileTrue(m_elevator.moveUp());
        // m_driverController.button(ButtonConstants.xboxA).whileTrue(m_elevator.moveDown());
        // m_driverController.button(ButtonConstants.xboxB).whileTrue(m_elevator.stop());

    
        new Trigger(() -> m_driverController.getHID().getPOV() == 0)
            .onTrue(m_elevator.moveUp());
        
        new Trigger(() -> m_driverController.getHID().getPOV() == 180)
            .onTrue(m_elevator.moveDown());
        
        new Trigger(() -> m_driverController.getHID().getPOV() == 90)
            .onTrue(m_elevator.stop());
        new Trigger(() -> m_driverController.getHID().getPOV() ==270)
            .onTrue(AlgaeIntake.runAlgaeRoller(-0.25));

        // Hang System
        // m_operatorController.rightTrigger()
        //     .whileTrue(new RunCommand(() -> m_hang.hangDown(), m_hang))
        //     .onFalse(new InstantCommand(() -> m_hang.stop()));

        // m_operatorController.leftTrigger()
        //     .whileTrue(new RunCommand(() -> m_hang.hangUp(), m_hang))
        //     .onFalse(new InstantCommand(() -> m_hang.stop()));

        // // D-Pad Up
        // m_operatorController.povUp().onTrue(new InstantCommand(() -> {
        //     if (speedMultiplier == 0.7) {
        //         speedMultiplier = 0.3; // Switch to slow speed
        //     } else {
        //         speedMultiplier = 0.7; // Switch to fast speed
        //     }
        //     System.out.println("Elevator Speed Changed: " + speedMultiplier);
        // }));
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            // m_elevator.moveTo(2.5),
            new RunCommand(() -> m_robotDrive.drive(0.2, 0, 0, false), m_robotDrive)
                .withTimeout(3),
            new InstantCommand(() -> m_robotDrive.drive(0.0, 0, 0, false)), // Stop movement
            coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralOuttakeSpeeds).withTimeout(2),

        
            new InstantCommand(() -> coralIntake.setCoralIntakeRoller(0))
        );
    }

    public double getElevatorHeight() {
        return m_elevator.getPositionInches();
    }
}
