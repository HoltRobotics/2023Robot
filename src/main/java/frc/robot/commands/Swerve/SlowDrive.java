// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SlowDrive extends CommandBase {
  private final Swerve m_swerve;

  /** Creates a new SlowDrive2. */
  public SlowDrive(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.setSpeedReducer(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.setSpeedReducer(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
