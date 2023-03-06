// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SlowDrive extends CommandBase {
  private final Swerve m_swerve; // The subsystem needed to control the Swerve.

  /**
   * A command that makes the robot drive at half speed.
   * @param swerve The Swerve Subsystem
   */
  public SlowDrive(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve; // Passes the subsystem to the rest of the command.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.setSpeedReducer(0.25); // Sets the speed cap to half speed.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.setSpeedReducer(1); // Sets the speed cap back to full speed.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
