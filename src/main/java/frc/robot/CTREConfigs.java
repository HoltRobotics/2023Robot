package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants.SwerveConstants;

public final class CTREConfigs {
    public TalonFXConfiguration m_swerveAngleFXConfig;
    public TalonFXConfiguration m_swerveDriveFXConfig;
    public CANCoderConfiguration m_swerveCanCoderConfig;

    public CTREConfigs(){
        m_swerveAngleFXConfig = new TalonFXConfiguration();
        m_swerveDriveFXConfig = new TalonFXConfiguration();
        m_swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveConstants.angleEnableCurrentLimit, 
            SwerveConstants.angleContinuousCurrentLimit, 
            SwerveConstants.anglePeakCurrentLimit, 
            SwerveConstants.anglePeakCurrentDuration);

        m_swerveAngleFXConfig.slot0.kP = SwerveConstants.angleKP;
        m_swerveAngleFXConfig.slot0.kI = SwerveConstants.angleKI;
        m_swerveAngleFXConfig.slot0.kD = SwerveConstants.angleKD;
        m_swerveAngleFXConfig.slot0.kF = SwerveConstants.angleKF;
        m_swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            SwerveConstants.driveEnableCurrentLimit, 
            SwerveConstants.driveContinuousCurrentLimit, 
            SwerveConstants.drivePeakCurrentLimit, 
            SwerveConstants.drivePeakCurrentDuration);

        m_swerveDriveFXConfig.slot0.kP = SwerveConstants.driveKP;
        m_swerveDriveFXConfig.slot0.kI = SwerveConstants.driveKI;
        m_swerveDriveFXConfig.slot0.kD = SwerveConstants.driveKD;
        m_swerveDriveFXConfig.slot0.kF = SwerveConstants.driveKF;        
        m_swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        m_swerveDriveFXConfig.openloopRamp = SwerveConstants.openLoopRamp;
        m_swerveDriveFXConfig.closedloopRamp = SwerveConstants.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        m_swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        m_swerveCanCoderConfig.sensorDirection = SwerveConstants.canCoderInvert;
        m_swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        m_swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}