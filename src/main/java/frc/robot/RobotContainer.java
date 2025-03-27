package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HangSubsystem;

public class RobotContainer {
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();
    private final HangSubsystem m_hang = new HangSubsystem(); 

    // Controllers
    private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kSecondaryControllerPort);

    // Speed multipliers
    private double speedMultiplier = 0.7;

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

    public boolean shouldSwitchCameras() {
        boolean current = m_operatorController.button(ButtonConstants.xboxA).getAsBoolean();
        boolean shouldSwitch = current && !lastCameraSwitchState;
        lastCameraSwitchState = current;
        return shouldSwitch;
    }        

    private void configureButtonBindings() {

        // MAIN KEYBINDS
        m_driverController.leftTrigger()
            .whileTrue(coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralIntakeSpeeds))
            .onFalse(new InstantCommand(() -> coralIntake.setCoralIntakeRoller(0))); // Stops when released

        m_driverController.rightTrigger()
            .whileTrue(coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralOuttakeSpeeds))
            .onFalse(new InstantCommand(() -> coralIntake.setCoralIntakeRoller(0))); // Stops when released

        m_driverController.leftBumper()
            .whileTrue(coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralIntakeSpeeds * 0.5))
            .onFalse(new InstantCommand(() -> coralIntake.setCoralIntakeRoller(0))); // Stops when released


        // OPPERATOR KEYBINDS
                

        // Elevator & Hang
        m_operatorController.button(ButtonConstants.xboxY).onTrue(m_elevator.moveTo(ElevatorConstants.vL3Height));
        m_operatorController.button(ButtonConstants.xboxB).onTrue(m_elevator.moveTo(ElevatorConstants.vL2Height));
        m_operatorController.button(ButtonConstants.xboxA).onTrue(m_elevator.moveTo(ElevatorConstants.vL1Height));
        m_operatorController.button(ButtonConstants.xboxX).onTrue(m_elevator.moveTo(ElevatorConstants.vL4Height));

        // Hang System
        m_operatorController.rightTrigger()
            .whileTrue(new RunCommand(() -> m_hang.hangDown(), m_hang))
            .onFalse(new InstantCommand(() -> m_hang.stop()));

        m_operatorController.leftTrigger()
            .whileTrue(new RunCommand(() -> m_hang.hangUp(), m_hang))
            .onFalse(new InstantCommand(() -> m_hang.stop()));

        // D-Pad Up
        m_operatorController.povUp().onTrue(new InstantCommand(() -> {
            if (speedMultiplier == 0.7) {
                speedMultiplier = 0.3; // Switch to slow speed
            } else {
                speedMultiplier = 0.7; // Switch to fast speed
            }
            System.out.println("Elevator Speed Changed: " + speedMultiplier);
        }));
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            new RunCommand(() -> m_robotDrive.drive(0.2, 0, 0, false), m_robotDrive)
                .withTimeout(3),
            new InstantCommand(() -> m_robotDrive.drive(0.0, 0, 0, false)), // Stop movement
            coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralOuttakeSpeeds).withTimeout(2),
            new InstantCommand(() -> coralIntake.setCoralIntakeRoller(0))
        );
    }
}
