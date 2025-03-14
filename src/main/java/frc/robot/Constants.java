package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

// import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
    // 🔹 Drive system constants
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
        public static final int kRearLeftDrivingCanId = 4;
        public static final int kFrontRightDrivingCanId = 2;
        public static final int kRearRightDrivingCanId = 1;

        public static final int kFrontLeftTurningCanId = 6;
        public static final int kRearLeftTurningCanId = 3;
        public static final int kFrontRightTurningCanId = 7;
        public static final int kRearRightTurningCanId = 8;

        public static final boolean kGyroReversed = false;
    }

    // 🔹 Swerve module constants
    public static final class ModuleConstants {
        public static final int kDrivingMotorPinionTeeth = 14;
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
    }

    // 🔹 Operator input constants
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kSecondaryControllerPort = 1;
        public static final double kDriveDeadband = 0.15;
    }

    // 🔹 Autonomous settings
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

    // 🔹 Elevator system constants
    public static final class ElevatorConstants {
        public static final int elevatorUpperMotorID = 9;

        public static final double kElevatorDefaultTolerance = 1.0;

        // ✅ Make sure these heights exist and are properly defined
        public static final double vL1Height = 0.0; // Base Level
        public static final double vL2Height = 3.17; // Level 2
        public static final double vL3Height = 19.04; // Level 3
        public static final double vL4Height = 43.32; // Level 4

        // ✅ Gear ratio and conversion factors
        public static final double kGearRatio = 0.255;
        public static final double kPositionConversionFactor = 1.0;
        public static final double kZeroOffset = 0.0;
        public static final double kEncoderCountsPerRotation = 42;
        public static final double kMaxSpeedPercentage = 0.5;

        // ✅ PID Constants
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final int kMaxLevel = 4;
        public static final int kMinLevel = 0;
    }

    // 🔹 Motor constants for NEO motors
    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }

    // 🔹 Controller button mappings
    public static final class ButtonConstants {
        public static final int xboxA = 1;
        public static final int xboxB = 2;
        public static final int xboxX = 3;
        public static final int xboxY = 4;

        public static final int xboxLB = 5;
        public static final int xboxRB = 6;
        public static final int xboxMAPS = 7;
        public static final int xboxLINES = 8;
        public static final int xboxLeftJoystickDown = 9;
        public static final int xboxRightJoystickDown = 10;
    }

    // 🔹 Drivebase constants
    public static final class DrivebaseConstants {
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    // 🔹 Intake system constants
    public static final class IntakeConstants {
        public static final int algaeUpperMotorID = 21;
        public static final int coralMotorID = 11;

        // ✅ Define Coral Intake Speeds
        public static final double CoralIntakeSpeeds = -0.05; // Adjust value as needed
        public static final double CoralOuttakeSpeeds = -0.2; // Adjust value as needed

        public static final double AlgaeIntakeSpeed = 0.1;
        public static final double AlgaeOuttakeSpeed = -0.1;
        public static final double CoralIntakeSpeed = -0.1;
        public static final double CoralOuttakeSpeed = -0.3;

        public static final double kIntakeReduction = 0;
    }

    // 🔹 Robot physical properties
    public static final class RobotPhysicalConstants {
        public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32 lbs converted to kg
        public static final double ROBOT_HEIGHT_METERS = Units.inchesToMeters(8); // Chassis height if needed
        public static final double LOOP_TIME = 0.13; // 20ms + 110ms Spark MAX velocity lag
        public static final double MAX_SPEED = (4.4 * 0.5); // Max limit speed is halved
    }

    // 🔹 Operator control constants
    public static final class OperatorConstants {
        public static final double DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static final class HangConstants {
        public static final int HangMotorID = 10;
    }
}
