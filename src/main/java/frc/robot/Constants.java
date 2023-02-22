package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final int kDriverPort = 0;
    public static final int kOperatorPort = 1;

    public static final double kMaxRobotHeight = 2; //TODO: get right height
    public static final double kMinRobotHeight = 0.1;

    public static final class ElevatorConstants {
        public static final int elevatorMotorID = 9;
        
        public static final double kStowHeight = 0; //TODO: get right heights
        public static final double kStage1Height = 0.74; 
        public static final double kStage2Height = 0.1458;
        public static final double kStage3Height = 0;
        public static final double kMaxHeight = 0.91;

        public static final double kBackWheelsOffset = 0.25; //TODO: get right distances
        public static final double kFrontWheelsOffset = 0.25;

        //TODO: tune theses
        public static final double kP = 200;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class ArmConstants {
        public static final int armMotorID = 10;

        public static final double kStowAngle = 0; //TODO: get the right angles
        public static final double kStage1Angle = 134;
        public static final double kStage2Angle = 32.43;
        public static final double kStage3Angle = 0;
        public static final double kMaxAngle = 180;
        public static final double kClawHeightOffset = 0.3; //TODO: get the right height

        public static final double kClawLenghtOffset = 0.25; //TODO: get right distances
        public static final double kArmLenght = 1;

        public static final double kP = 0.25;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class PneumaticsConstants {
        public static final int kClawTiltForward = 0;
        public static final int kClawTiltReverse = 1;
        public static final int kClaw = 2;
        public static final int kBuddyForward = 3;
        public static final int kBuddyReverse = 4;

    }

    public static final class SwerveConstants {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final double kDistanceFromTagMeters = 1;

        public static final COTSFalconSwerveConstants chosenModule =
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.5);
        public static final double wheelBase = Units.inchesToMeters(24.5);
        public static final double wheelCircumference = chosenModule.m_wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.m_driveGearRatio;
        public static final double angleGearRatio = chosenModule.m_angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.m_angleMotorInvert;
        public static final boolean driveMotorInvert = !chosenModule.m_driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.m_canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.m_angleKP;
        public static final double angleKI = chosenModule.m_angleKI;
        public static final double angleKD = chosenModule.m_angleKD;
        public static final double angleKF = chosenModule.m_angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = Units.feetToMeters(16.3);
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /*
         * When calibrating CANCoders have the bevel gear point left
         */
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(130.69);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(32.87);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(98.52);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(129.55);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        //TODO: when the other modules are built lets set the CAN IDs ahead of time so when we swap them out its easier in code.
        /* Extra Module - Module 4 */
        public static final class Mod4 {
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 5;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Extra Extra Module - Module 5 */
        public static final class Mod5 {
            public static final int driveMotorID = 22;
            public static final int angleMotorID = 23;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
}
