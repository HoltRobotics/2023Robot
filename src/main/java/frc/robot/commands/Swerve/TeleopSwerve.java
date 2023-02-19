package frc.robot.commands.Swerve;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private DoubleSupplier m_translation;
    private DoubleSupplier m_strafe;
    private DoubleSupplier m_rotation;
    private BooleanSupplier m_robotCentric;
    private Swerve m_swerve;

    public TeleopSwerve(DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation, BooleanSupplier robotCentric, Swerve swerve) {
        m_translation = translation;
        m_strafe = strafe;
        m_rotation = rotation;
        m_robotCentric = robotCentric;
        m_swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(m_translation.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(m_strafe.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(m_rotation.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        m_swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), 
            rotationVal * SwerveConstants.maxAngularVelocity, 
            !m_robotCentric.getAsBoolean(), 
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stopDrive();
    }
}