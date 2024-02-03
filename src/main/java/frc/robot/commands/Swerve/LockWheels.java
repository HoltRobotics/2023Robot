// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class LockWheels extends CommandBase {
  private final Swerve m_swerve; // The subsystem needed to control the Swerve.
  private final SwerveModuleState[] m_states = { // The states of each of the wheels that we want to set.
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)), // Front Left
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)), // Front Right
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)), // Back Left
    new SwerveModuleState(0, Rotation2d.fromDegrees(45)) // Back Right
  };

  /** Creates a new LockWheels. */
  public LockWheels(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve; // Passes the subsystem to the rest of the command.
    addRequirements(m_swerve); // Stops all other commands using the Swerve subsystem.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.setModuleStates(m_states); // Sets the module states to the states we want.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
