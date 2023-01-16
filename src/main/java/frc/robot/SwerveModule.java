package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    public int m_moduleNumber;
    private Rotation2d m_angleOffset;
    private Rotation2d m_lastAngle;

    private TalonFX m_angleMotor;
    private TalonFX m_driveMotor;
    private CANCoder m_angleEncoder;

    public double m_CANcoderInitTime = 0.0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        m_moduleNumber = moduleNumber;
        m_angleOffset = moduleConstants.m_angleOffset;
        
        /* Angle Encoder Config */
        m_angleEncoder = new CANCoder(moduleConstants.m_cancoderID);
        configAngleEncoder();
        // configAngleEncoder(); //TODO: See if encoders work. Delete if they do.
        // configAngleEncoder();

        /* Angle Motor Config */
        m_angleMotor = new TalonFX(moduleConstants.m_angleMotorID);
        configAngleMotor();
        // configAngleMotor(); //TODO: See if encoders work. Delete if they do.
        // configAngleMotor();

        /* Drive Motor Config */
        m_driveMotor = new TalonFX(moduleConstants.m_driveMotorID);
        configDriveMotor();
        // configDriveMotor(); //TODO: See if encoders work. Delete if they do.
        // configDriveMotor();

        m_lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? m_lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        m_angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        m_lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(m_angleEncoder.getAbsolutePosition());
    }

    private void waitForCanCoder(){
        /*
         * Wait for up to 1000 ms for a good CANcoder signal.
         *
         * This prevents a race condition during program startup
         * where we try to synchronize the Falcon encoder to the
         * CANcoder before we have received any position signal
         * from the CANcoder.
         */
        for (int i = 0; i < 100; ++i) {
            m_angleEncoder.getAbsolutePosition();
            if (m_angleEncoder.getLastError() == ErrorCode.OK) {
                break;
            }
            Timer.delay(0.010);            
            m_CANcoderInitTime += 10;
        }
    }

    public void resetToAbsolute(){
        waitForCanCoder();

        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - m_angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        m_angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        m_angleEncoder.configFactoryDefault();
        m_angleEncoder.configAllSettings(Robot.m_ctreConfigs.m_swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        m_angleMotor.configFactoryDefault();
        m_angleMotor.configAllSettings(Robot.m_ctreConfigs.m_swerveAngleFXConfig);
        m_angleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        m_angleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        Timer.delay(0.05);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(Robot.m_ctreConfigs.m_swerveDriveFXConfig);
        m_driveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        m_driveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}