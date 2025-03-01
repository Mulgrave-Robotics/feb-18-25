package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);
    private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();

    private final double speedMultiplier = 0.5;

    public RobotContainer() {
        configureButtonBindings();

        // Default driving control using Xbox controller
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband)
                                        * speedMultiplier, // Y-axis:
                                // forward/backward
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)
                                        * speedMultiplier, // X-axis:
                                // left/right
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)
                                        * speedMultiplier, // Right
                                // thumbstick:
                                // rotation
                                true),
                        m_robotDrive));
    }

    private void configureButtonBindings() {
        // Map Xbox controller buttons to elevator commands
        // m_driverController.button(ButtonConstants.xboxY).onTrue(m_elevator.goToLevel(1));
        // // Y Button
        // m_driverController.button(Button.kB.value).onTrue(m_elevator.goToLevel(2));
        // // B Button
        // m_driverController.button(Button.kX.value).onTrue(m_elevator.goToLevel(0));
        // // X Button (Reset)

        m_driverController.button(ButtonConstants.xboxY).onTrue(m_elevator.moveTo(ElevatorConstants.vL4Height));
        // m_driverController.button(ButtonConstants.xboxX).onTrue(m_elevator.moveTo(ElevatorConstants.vL3Height));
        m_driverController.button(ButtonConstants.xboxB).onTrue(m_elevator.moveTo(ElevatorConstants.vL2Height));
        m_driverController.button(ButtonConstants.xboxA).onTrue(m_elevator.moveTo(ElevatorConstants.vL1Height));

        // Example: Adding button for autonomous/other control (if needed)
        m_driverController.button(ButtonConstants.xboxRB)
                .onTrue(coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralIntakeSpeeds));

        m_driverController.button(ButtonConstants.xboxLB)
                .onTrue(coralIntake.setCoralIntakeRoller(Constants.IntakeConstants.CoralOuttakeSpeeds));

        m_driverController.button(ButtonConstants.xboxX)
                .onTrue(coralIntake.setCoralIntakeRoller(0));

        // For example, RB and LB could be mapped to different drive modes or additional
        // control commands.
    }

    // Autonomous command: Move forward for 3 seconds
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                new RunCommand(() -> m_robotDrive.drive(0.2, 0, 0, false), m_robotDrive)
                        .withTimeout(3),
                new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
    }
}
