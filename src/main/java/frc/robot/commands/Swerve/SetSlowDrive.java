// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

public class SetSlowDrive extends InstantCommand {
  private final Swerve m_drive;
  private final boolean m_isSlow;

  public SetSlowDrive(Swerve drive, boolean isSlow) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_isSlow = isSlow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_isSlow) {
      m_drive.setSpeedReducer(0.5);
    } else {
      m_drive.setSpeedReducer(1);
    }
  }
}
