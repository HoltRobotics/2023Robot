// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.Elevator;

public class Up extends CommandBase {
  private final Elevator m_lift;
  private boolean m_pastLimit = false;

  /** Creates a new Up. */
  public Up(Elevator lift) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_lift = lift;
    addRequirements(m_lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lift.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lift.up();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_pastLimit) {
      m_lift.setHeight(ElevatorConstants.kMaxHeight);
    } else{
      m_lift.setHeight(m_lift.getHeight());
    }
    m_lift.enable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_lift.getHeight() > ElevatorConstants.kMaxHeight) {
      m_pastLimit = true;
      return true;
    } else{
      return false;
    }
  }
}
