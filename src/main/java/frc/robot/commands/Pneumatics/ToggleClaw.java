// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Pneumatics;

public class ToggleClaw extends InstantCommand {
  private final Pneumatics m_air;

  public ToggleClaw(Pneumatics air) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_air = air;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_air.getClawState() == Value.kForward) {
      m_air.setClawState(Value.kReverse);
    } else {
      m_air.setClawState(Value.kForward);
    }
  }
}