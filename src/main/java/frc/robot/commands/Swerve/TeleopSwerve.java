package frc.robot.commands.Swerve;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private DoubleSupplier m_translation; // Used to store the Translation value.
    private DoubleSupplier m_strafe; // Used to store the Strafe value.
    private DoubleSupplier m_rotation; // Used to store the Rotation value.
    private BooleanSupplier m_robotCentric; // Used to store if using robot centric drive or not.
    private Swerve m_swerve; // Subsystem needed to control the swerve drive.

    /**
     * The default driving command.
     * @param translation The Translation Value
     * @param strafe The Strafe Value
     * @param rotation The Rotation Value
     * @param robotCentric If using robot centric drive or not.
     * @param swerve The Swerve Subsystem
     */
    public TeleopSwerve(DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation, BooleanSupplier robotCentric, Swerve swerve) {
        m_translation = translation; // Passes the given value to the rest of the command.
        m_strafe = strafe; // Passes the given value to the rest of the command.
        m_rotation = rotation; // Passes the given value to the rest of the command.
        m_robotCentric = robotCentric; // Passes the given value to the rest of the command.
        m_swerve = swerve; // Passes the given subsystem to the rest of the command.
        addRequirements(swerve); // Stops all other commands that are using the Swerve Subsystem.
    }

    @Override
    public void execute() {
        // Takes the values and applies a deadband to them.
        double translationVal = MathUtil.applyDeadband(m_translation.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(m_strafe.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(m_rotation.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        m_swerve.drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxSpeed), // Takes the translation and strafe values and normalizes them to the max robot speed.
            rotationVal * SwerveConstants.maxAngularVelocity, // Takes the rotation value and normalize's it to the max rotation speed.
            !m_robotCentric.getAsBoolean(), // If we are using robot centric or field centric.
            true // If we are using feedback to correct the steering motors.
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stopDrive(); // Stops all the wheels.
    }
}