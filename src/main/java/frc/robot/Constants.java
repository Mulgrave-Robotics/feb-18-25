package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kTrackWidth = Units.inchesToMeters(27);
    public static final double kWheelBase = Units.inchesToMeters(27);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 1;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    public static final int kDrivingMotorPinionTeeth = 14;

    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ElevatorConstants {
    public static final int kLeftMotorCanId = 9;
    public static final int kRightMotorCanId = 10;

    // public static final double kGearRatio = 4.4 * 300;
    // Calculated by Lok
    public static final double kGearRatio = 0.2608;
    
    // up to level 2 for now
    public static final int kMaxLevel = 2;
    public static final int kMinLevel = 0;
    public static final double kBaseHeight = 18.0;
    public static final double kLevel1Height = 31.875;
    public static final double kLevel2Height = 47.625;
    public static final double kMaxHeight = 72.0;
    public static final double kElevatorDefaultTolerance = 1.0;

    public static final double kPositionConversionFactor = 1.0; // Adjust based on encoder specs
    public static final double kZeroOffset = 0.0; // Adjust if needed

    public static final double kEncoderCountsPerRotation = 42;
    public static final double kMaxSpeedPercentage = 0.5;

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}