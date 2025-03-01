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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

public class RobotContainer {
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        configureButtonBindings();

        // ✅ Default driving control using joystick
        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getTwist(), OIConstants.kDriveDeadband),
                    true),
                m_robotDrive
            )
        );
    }

    private void configureButtonBindings() {
        // ✅ X-Configuration for Swerve Drive
        new JoystickButton(m_driverController, Button.kR1.value)
            .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

        // ✅ Elevator Controls
        new JoystickButton(m_driverController, 1).onTrue(m_elevator.goToLevel(1)); // A Button
        new JoystickButton(m_driverController, 2).onTrue(m_elevator.goToLevel(2)); // B Button
        new JoystickButton(m_driverController, 4).onTrue(m_elevator.goToLevel(0)); // Y Button (Reset)
    }

    // ✅ Autonomous command: Move forward for 3 seconds
    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
            new RunCommand(() -> m_robotDrive.drive(0.2, 0, 0, false), m_robotDrive)
                .withTimeout(3), 
            new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false))
        );
    }
}
