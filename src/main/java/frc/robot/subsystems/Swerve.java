package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_swerveMods;
    public Pigeon2 m_gyro;

    public Swerve() {
        m_gyro = new Pigeon2(Constants.Swerve.pigeonID);
        m_gyro.configFactoryDefault();
        zeroGyro();

        m_swerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        m_swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

        for(SwerveModule mod : m_swerveMods) {
            DriverStation.reportError("CANCoder on Module " + mod.m_moduleNumber + " took " + mod.m_CANcoderInitTime + " ms to be ready.", false);  //TODO: See if encoders work. Delete if they do.
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : m_swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : m_swerveMods){
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : m_swerveMods){
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : m_swerveMods){
            positions[mod.m_moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        m_gyro.setYaw(0);
    }

    public void resetEncoders() {
        for(SwerveModule mod : m_swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw()) : Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    public void stopDrive() {
        drive(new Translation2d(), 0, false, false);
    }

    @Override
    public void periodic(){
        // if (DriverStation.isDisabled()) { //TODO: See if encoders work. Delete if they do.
        //     resetEncoders();
        // }

        m_swerveOdometry.update(getYaw(), getModulePositions());  

        for(SwerveModule mod : m_swerveMods){
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}