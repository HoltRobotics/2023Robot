// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class NewBalance extends CommandBase {
  private final Swerve m_drive;

  /** Creates a new NewBalance. */
  public NewBalance(Swerve drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_drive.getPitch() >= 7) {
      m_drive.drive(new Translation2d(0.55, 0), 0, false, true);
    } else if(m_drive.getPitch() <= -7) {
      m_drive.drive(new Translation2d(-0.55, 0), 0, false, true);
    } else {
      m_drive.drive(new Translation2d(), 0, false, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(new Translation2d(), 0, false, true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
