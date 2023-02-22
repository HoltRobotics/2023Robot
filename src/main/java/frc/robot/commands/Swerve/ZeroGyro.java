// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class ZeroGyro extends InstantCommand {
  private Swerve m_swerve; // Subsystem needed to control the swerve drive.

  /**
   * A Command that zeros the gyro to reset field centric driving.
   * @param swerve The Swerve Subsystem
   */
  public ZeroGyro(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve; // Passes the given subsystem to the rest of the command.
    addRequirements(m_swerve); // Stops all other commands that are using the Swerve Subsystem.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.zeroGyro(); // Zeros the Gyro.
  }
}
