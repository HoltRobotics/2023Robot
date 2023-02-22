// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.Arm;

public class DownArm extends CommandBase {
  private final Arm m_arm; // Subsystem needed to control the arm.
  private boolean m_pastLimit = false; // Used to track if the arm goes past its limit.

  /**
   * Command that forces the arm to rotate down.
   * When the command ends, the new location will become the new setpoint.
   * @param arm The Arm Subsystem
   */
  public DownArm(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm; // Passes the given subsystem to the rest of the command.
    addRequirements(m_arm); // Stops all other commands using the Arm subsystem.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.disable(); // Disables the PID controller so it won't fight back.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.down(); // Calls for the arm to run down.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_pastLimit) { // Checks to see if the arm went past its max.
      m_arm.setAngle(ArmConstants.kMaxAngle); // If it did, set it back to the max.
    } else{
      m_arm.setAngle(m_arm.getAngle()); // If it didn't set the PID setpoint to the new angle.
    }
    m_arm.enable(); // Re-enables the PID controller.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_arm.getAngle() > ArmConstants.kMaxAngle) { // Checks to see if the arm is past the max angle.
      m_pastLimit = true; // If it is, tells the command that it went past the limit.
      return true; // Ends the command.
    } else{
      return false;// If its not, let the command keep running.
    }
  }
}
