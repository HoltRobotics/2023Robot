// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorProfiled;

public class LowerCone extends InstantCommand {
  private final ElevatorProfiled m_lift;

  /** Creates a new LowerCone. */
  public LowerCone(ElevatorProfiled lift) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.setHeight(m_lift.getHeight() - ElevatorConstants.kLowerConeHeight);
  }
}
