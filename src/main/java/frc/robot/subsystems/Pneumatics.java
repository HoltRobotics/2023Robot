// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
  private Compressor m_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private final GenericEntry m_isCompressorOn;
  private final GenericEntry m_isClawOpen;
  private final GenericEntry m_isClawTilted;
  private final GenericEntry m_isBuddyDown;

  private final Solenoid m_clawSol = new Solenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.kClaw);
  private final DoubleSolenoid m_clawTiltSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.kClawTiltForward, PneumaticsConstants.kClawTiltReverse);
  private final DoubleSolenoid m_buddySol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.kBuddyForward, PneumaticsConstants.kBuddyReverse);

  private boolean m_clawState;
  private Value m_tiltState;
  private Value m_buddyState;

  /** Creates a new Claw. */
  public Pneumatics() {
    m_compressor.enableDigital();
    m_isCompressorOn = m_tab.add("Compressor", isCompressorOn()).withWidget(BuiltInWidgets.kBooleanBox).withPosition(3, 0).getEntry();
    m_isClawOpen = m_tab.add("Claw Open",getClawState()).withWidget(BuiltInWidgets.kBooleanBox).withPosition(4, 0).getEntry();
    m_isClawTilted = m_tab.add("Is Claw Tilted", valueToBool(getTiltState())).withWidget(BuiltInWidgets.kBooleanBox).withPosition(5, 0).getEntry();
    m_isBuddyDown = m_tab.add("Forks Down", valueToBool(getBuddyState())).withWidget(BuiltInWidgets.kBooleanBox).withPosition(6, 0).getEntry();
    setClawState(false);
    setTiltState(Value.kReverse);
    setBuddyState(Value.kOff);
  }

  public boolean getClawState() {
    return m_clawState;
  }

  public Value getTiltState() {
    return m_tiltState;
  }

  public Value getBuddyState() {
    return m_buddyState;
  }

  public void setClawState(Boolean value) {
    m_clawSol.set(value);
    m_clawState = value;
  }

  public void setTiltState(Value value) {
    m_clawTiltSol.set(value);
    m_tiltState = value;
  }

  public void setBuddyState(Value value) {
    m_buddySol.set(value);
    m_buddyState = value;
  }

  public boolean valueToBool(Value value) {
    if(value == Value.kReverse || value == Value.kOff) {
      return false;
    } else {
      return true;
    }
  }

  public boolean isCompressorOn() {
    return m_compressor.isEnabled();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_isCompressorOn.setBoolean(isCompressorOn());
    m_isClawOpen.setBoolean(getClawState());
    m_isClawTilted.setBoolean(valueToBool(getTiltState()));
    m_isBuddyDown.setBoolean(valueToBool(getBuddyState()));
  }
}
