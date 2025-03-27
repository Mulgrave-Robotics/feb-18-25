package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset);
    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset);
    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset);
    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset);

    final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

    // ✅ Acceleration Control Variables
    private double prevXSpeed = 0;
    private double prevYSpeed = 0;
    private double prevRotSpeed = 0;
    private static final double MAX_ACCELERATION = 0.1; // Limits acceleration increase per loop

    // Initialize the odometry with a starting angle of 180 degrees
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(180),  // Set starting angle to 180 degrees
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    public DriveSubsystem() {
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
        m_gyro.reset();
    }

    @Override
    public void periodic() {
        m_odometry.update(
            Rotation2d.fromDegrees(m_gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            });

        // Print to console for debugging
        System.out.println("AHRS Gyro Angle: " + m_gyro.getAngle());
        System.out.println("AHRS Gyro Yaw: " + m_gyro.getYaw());

        // Log to SmartDashboard
        SmartDashboard.putNumber("AHRS Gyro Angle", m_gyro.getAngle());
        SmartDashboard.putNumber("AHRS Gyro Yaw", m_gyro.getYaw());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_gyro.reset();
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(180),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        xSpeed = applyAccelerationRamp(prevXSpeed, xSpeed);
        ySpeed = applyAccelerationRamp(prevYSpeed, ySpeed);
        rot = applyAccelerationRamp(prevRotSpeed, rot);

        prevXSpeed = xSpeed;
        prevYSpeed = ySpeed;
        prevRotSpeed = rot;

        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getYaw() + 180))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    private double applyAccelerationRamp(double previousSpeed, double desiredSpeed) {
        double speedDifference = desiredSpeed - previousSpeed;
        return Math.abs(speedDifference) > MAX_ACCELERATION
            ? previousSpeed + Math.signum(speedDifference) * MAX_ACCELERATION
            : desiredSpeed;
    }
}
