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
import frc.robot.subsystems.coralIntake;

public class RobotContainer {
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

        private final CommandXboxController m_driverController = new CommandXboxController(
                        OIConstants.kDriverControllerPort);
        private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();
        // private final coralIntake coralIntakeCommand = new coralIntake(coralIntake);

        private final double speedMultiplier = 0.7;

        public RobotContainer() {
                configureButtonBindings();

                // Default driving control using Xbox controller
                m_robotDrive.setDefaultCommand(
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OIConstants.kDriveDeadband)
                                                                                * speedMultiplier, // Y-axis:
                                                                                                   // forward/backward
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OIConstants.kDriveDeadband)
                                                                                * speedMultiplier, // X-axis: left/right
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OIConstants.kDriveDeadband)
                                                                                * speedMultiplier, // Right stick:
                                                                                                   // rotation
                                                                false),
                                                m_robotDrive));
        }

        private void configureButtonBindings() {
                // Elevator Controls
                m_driverController.button(ButtonConstants.xboxY).onTrue(m_elevator.moveTo(ElevatorConstants.vL3Height));
                m_driverController.button(ButtonConstants.xboxB).onTrue(m_elevator.moveTo(ElevatorConstants.vL2Height));
                m_driverController.button(ButtonConstants.xboxA).onTrue(m_elevator.moveTo(ElevatorConstants.vL1Height));

                // Coral Intake Controls

                m_driverController.button(ButtonConstants.xboxRB)
                                .whileTrue(coralIntake
                                                .setCoralIntakeRoller(Constants.IntakeConstants.CoralIntakeSpeeds));
                m_driverController.button(ButtonConstants.xboxLB)
                                .whileTrue(coralIntake
                                                .setCoralIntakeRoller(Constants.IntakeConstants.CoralOuttakeSpeeds));
                m_driverController.button(ButtonConstants.xboxX).onTrue(coralIntake.setCoralIntakeRoller(0));
        }

        // Autonomous command: Move forward for 3 seconds
        public Command getAutonomousCommand() {
                return new SequentialCommandGroup(
                                new RunCommand(() -> m_robotDrive.drive(0.2, 0, 0, false), m_robotDrive)
                                                .withTimeout(3),
                                new InstantCommand(() -> m_robotDrive.drive(0.0, 0, 0, false)), // Stop movement
                                coralIntake
                                                .setCoralIntakeRoller(Constants.IntakeConstants.CoralOuttakeSpeeds)
                                                .withTimeout(2),
                                new InstantCommand(() -> coralIntake.setCoralIntakeRoller(0))

                );
        }
}
