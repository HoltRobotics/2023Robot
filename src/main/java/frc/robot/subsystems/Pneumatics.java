// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  private Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  private final DoubleSolenoid m_clawSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.kClawForward, Constants.Pneumatics.kClawReverse);
  private final DoubleSolenoid m_clawTiltSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.kClawTiltForward, Constants.Pneumatics.kClawTiltReverse);

  /** Creates a new Claw. */
  public Pneumatics() {
    m_compressor.enableDigital();
  }

  public void openClaw() {
    m_clawSol.set(Value.kForward);
  }

  public void closeClaw() {
    m_clawSol.set(Value.kReverse);
  }

  public void clawDown() {
    m_clawTiltSol.set(Value.kForward);
  }

  public void clawUp() {
    m_clawTiltSol.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: changes this to Shuffleboard
    SmartDashboard.putBoolean("Is On:", m_compressor.isEnabled());
  }
}
