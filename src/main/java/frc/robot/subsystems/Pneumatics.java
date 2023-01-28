// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
  private Compressor m_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private final GenericEntry m_isCompressorOn;

  private final DoubleSolenoid m_clawSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.kClawForward, Constants.Pneumatics.kClawReverse);
  private final DoubleSolenoid m_clawTiltSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.kClawTiltForward, Constants.Pneumatics.kClawTiltReverse);
  private final DoubleSolenoid m_buddySol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Pneumatics.kBuddyForward, Constants.Pneumatics.kBuddyReverse);

  /** Creates a new Claw. */
  public Pneumatics() {
    m_compressor.enableDigital();
    m_isCompressorOn = m_tab.add("Compressor", false).getEntry();
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

  public void buddyDown() {
    m_buddySol.set(Value.kForward);
  }

  public void buddyUp() {
    m_buddySol.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_tab.addBoolean("Is compressor on", () -> m_compressor.isEnabled());
    m_isCompressorOn.setBoolean(m_compressor.isEnabled());
  }
}
