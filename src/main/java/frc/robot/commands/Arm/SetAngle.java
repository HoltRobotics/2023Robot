// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmProfiled;

public class SetAngle extends InstantCommand {
  private double m_angle; // Used to store the given angle.
  private ArmProfiled m_arm; // Subsystem needed to control the arm.

  /**
   * Command to set the PID angle of the Arm.
   * @param angle Angle to set the arm too.
   * @param arm The Arm Subsystem
   */
  public SetAngle(double angle, ArmProfiled arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_angle = angle; // Passes the given angle to the rest of the command.
    m_arm = arm; // Passes the given subsystem to the rest of the command.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setInMotion(true);
    m_arm.setAngle(m_angle); // Sets the PID angle to the given angle.
  }
}
