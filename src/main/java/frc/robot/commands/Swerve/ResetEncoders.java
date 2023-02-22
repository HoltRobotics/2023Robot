// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class ResetEncoders extends InstantCommand {
  private Swerve m_swerve; // The subsystem needed to control the Swerve.

  /**
   * A command that resets the steering encoder back to the absolute encoder value.
   * @param swerve // The Swerve Subsystem
   */
  public ResetEncoders(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve; // Passes the subsystem to the rest of the command.
    addRequirements(m_swerve); // Stops all other commands using the Swerve subsystem.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.resetEncoders(); // Resets the encoders.
  }
}
