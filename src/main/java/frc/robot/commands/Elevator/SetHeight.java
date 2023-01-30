// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class SetHeight extends InstantCommand {
  private Elevator m_lift;
  private double m_height;

  public SetHeight(double height, Elevator lift) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_height = height;
    m_lift = lift;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.setHeight(m_height);
  }
}
