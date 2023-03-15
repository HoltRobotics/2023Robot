package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveDriveKinematics m_kinematics;
    public SwerveModule[] m_swerveMods;
    public Pigeon2 m_gyro;
    public double m_slowDrive;
    
    public Swerve() {
        m_gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
        m_gyro.configFactoryDefault();
        zeroGyro();

        m_swerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        Timer.delay(1.0);
        resetEncoders();

        m_slowDrive = 1;

        m_swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getModulePositions());
        m_kinematics = Constants.SwerveConstants.swerveKinematics;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX() * m_slowDrive, 
                                    translation.getY() * m_slowDrive, 
                                    rotation * m_slowDrive, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX() * m_slowDrive, 
                                    translation.getY() * m_slowDrive, 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for(SwerveModule mod : m_swerveMods){
            mod.setDesiredState(swerveModuleStates[mod.m_moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
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

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath, boolean isRed) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                if(isFirstPath) {
                    resetOdometry(traj.getInitialHolonomicPose());
                }
            }),
            new PPSwerveControllerCommand(
                traj,
                this::getPose,
                Constants.SwerveConstants.swerveKinematics,
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0),
                new PIDController(0, 0, 0),
                this::setModuleStates,
                isRed,
                this
            )
        );
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

    public void setKinematic(SwerveDriveKinematics kinematics) {
        m_kinematics = kinematics;
    }

    public void resetKinematic() {
        m_kinematics = Constants.SwerveConstants.swerveKinematics;
    }

    public void zeroGyro(){
        m_gyro.setYaw(0);
    }

    public void zeroGyro(double angle) {
        m_gyro.setYaw(angle);
    }

    public void resetEncoders() {
        for(SwerveModule mod : m_swerveMods) {
            mod.resetToAbsolute();
        }
    }

    public Rotation2d getYaw() {
        return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw()) : Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    public double getPitch() {
        return m_gyro.getRoll();
    }

    public void stopDrive() {
        drive(new Translation2d(), 0, false, false);
    }

    public void setSpeedReducer(double reduce) {
        m_slowDrive = reduce;
    }

    @Override
    public void periodic(){
        m_swerveOdometry.update(getYaw(), getModulePositions());
        // System.out.println(getPitch());
        // for(SwerveModule mod : m_swerveMods){
            // SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        // }
    }
}