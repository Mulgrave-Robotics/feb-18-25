// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private UsbCamera camera1;
  private UsbCamera camera2;
  private VideoSink server;
  private boolean usingCamera1 = true;
  Joystick joy1 = new Joystick(0);

  public Robot() {
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);

    camera1.setResolution(320, 240);
    camera2.setResolution(320, 240);

    camera1.setFPS(20);
    camera2.setFPS(20);

    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    camera2.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    server = CameraServer.getServer();
    server.setSource(camera1); 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // if (m_robotContainer.shouldSwitchCameras()) {
    //   usingCamera1 = !usingCamera1;
    //   server.setSource(usingCamera1 ? camera1 : camera2);
    //   System.out.println("📸 Switched to " + (usingCamera1 ? "Camera 1" : "Camera 2"));
    // }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("currentHeight", m_robotContainer.getElevatorHeight());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
