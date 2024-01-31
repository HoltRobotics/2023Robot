// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.*;
import frc.robot.subsystems.ElevatorProfiled;

public class Up extends Command {
  private final ElevatorProfiled m_lift; // Subsystem needed to control the elevator.
  private boolean m_pastLimit = false; // Used to track if the elevator goes past its limit.

  /**
   * Command that forces the elevator up.
   * When the command ends, the new location will become the new setpoint.
   * @param lift The Elevator Subsystem
   */
  public Up(ElevatorProfiled lift) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift; // Passes the given subsystem to the rest of the command.
    addRequirements(m_lift); // Stops all other commands using the Elevator subsystem.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.disable(); // Disables the PID controller so it won't fight back.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lift.up(); // Calls for the elevator to run up.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_pastLimit) { // Checks to see if the elevator went past its max.
      m_lift.setHeight(ElevatorConstants.kMaxHeight); // If it did, set it back to the max.
    } else{
      m_lift.setHeight(m_lift.getHeight()); // If it didn't, set the PID setpoint to the new height.
    }
    m_lift.enable(); // Re-Enables the PID controller.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_lift.getHeight() > ElevatorConstants.kMaxHeight) { // Checks to see if the elevator is past the max height.
      m_pastLimit = true; // If it is, tells the command that it went past the limit.
      return true; // Ends the command.
    } else{
      return false; // If its not, let the command keep running.
    }
  }
}
