// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmProfiled;

public class AnalongArmMove extends Command {
  private final ArmProfiled m_arm;
  private DoubleSupplier m_upSpeed;
  private DoubleSupplier m_downSpeed;

  /** Creates a new AnalongDown. */
  public AnalongArmMove(DoubleSupplier downSpeed, DoubleSupplier upSpeed, ArmProfiled arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_upSpeed = upSpeed;
    m_downSpeed = downSpeed;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_arm.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_upSpeed.getAsDouble() >= 0.1 || m_downSpeed.getAsDouble() >= 0.1) {
      m_arm.disable();
      m_arm.setSpeed((m_upSpeed.getAsDouble() - m_downSpeed.getAsDouble()) * 0.6);
      m_arm.setAngle(m_arm.getAngle());
    } else {
      m_arm.enable();
    }
    m_arm.periodic();
    // System.out.println(m_arm.isEnabled());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.enable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(m_upSpeed.getAsDouble() <= 0.1 || m_downSpeed.getAsDouble() <= 0.1) {
    //   return true;
    // } else {
    //   return false;
    // }
    return m_arm.m_inMotion;
  }
}
