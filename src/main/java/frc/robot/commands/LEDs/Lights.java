// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;

public class Lights extends CommandBase {
  private final LEDs m_led;

  /** Creates a new AutonLights. */
  public Lights(LEDs led) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_led = led;
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.isDisabled()) {
      m_led.setColorTeleOP(Alliance.Invalid);
    } else if(DriverStation.isAutonomous()) {
      m_led.setColorAuton(DriverStation.getAlliance());
    } else if (DriverStation.isTeleop()) {
      m_led.setColorTeleOP(DriverStation.getAlliance());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_led.setColorAuton(Alliance.Invalid);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
