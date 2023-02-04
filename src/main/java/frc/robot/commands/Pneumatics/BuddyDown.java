// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Pneumatics;

public class BuddyDown extends InstantCommand {
  private final Pneumatics m_air;

  public BuddyDown(Pneumatics air) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_air = air;
    addRequirements(m_air);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_air.setBuddyState(Value.kForward);
  }
}
