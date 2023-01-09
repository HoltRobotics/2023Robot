package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

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
            Constants.Swerve.angleEnableCurrentLimit, 
            Constants.Swerve.angleContinuousCurrentLimit, 
            Constants.Swerve.anglePeakCurrentLimit, 
            Constants.Swerve.anglePeakCurrentDuration);

        m_swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        m_swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        m_swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        m_swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        m_swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.driveEnableCurrentLimit, 
            Constants.Swerve.driveContinuousCurrentLimit, 
            Constants.Swerve.drivePeakCurrentLimit, 
            Constants.Swerve.drivePeakCurrentDuration);

        m_swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        m_swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        m_swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        m_swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;        
        m_swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        m_swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
        m_swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        m_swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        m_swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        m_swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        m_swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}