// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Pneumatics;

public class ToggleClaw extends InstantCommand {
  private final Pneumatics m_air; // The subsystem needed to control the Pneumatics.

  /**
   * Combo command that toggles the state of the claw. If the claw is open, it will close. If the Claw is closed, it will open.
   * @param air The Pneumatics Subsystem
   */
  public ToggleClaw(Pneumatics air) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_air = air; // Passes the given subsystem to the rest of the command.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_air.setClawState(!m_air.getClawState()); // Sets the claw state to the opposite of it's current state.
  }
}
