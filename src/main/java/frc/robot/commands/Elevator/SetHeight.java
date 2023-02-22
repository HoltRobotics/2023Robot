// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

public class SetHeight extends InstantCommand {
  private Elevator m_lift; // Subsystem needed to control the elevator.
  private double m_height; // Used to store the given height.

  /**
   * Command to set the PID height of the Elevator
   * @param height Height to set the elevator too.
   * @param lift The Elevator Subsystem
   */
  public SetHeight(double height, Elevator lift) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_height = height; // Passes the given height to the rest of the command.
    m_lift = lift; // Passes the given subsystem to the rest of the command.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.setHeight(m_height); // Sets the PID height to the given angle.
  }
}
