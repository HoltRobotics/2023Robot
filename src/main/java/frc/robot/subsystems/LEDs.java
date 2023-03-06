// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
  private final Spark m_blinkin = new Spark(LEDConstants.kBlinkinPort);
  private double m_color = 0.67;

  /** Creates a new LEDs. */
  public LEDs() {}
  
  // Sets the robot to the alliance color
  public void setColorTeleOP(Alliance color){
    if(color == Alliance.Blue){
      m_color = 0.87;
      m_blinkin.set(0.87);
    }else if(color == Alliance.Red){
      m_blinkin.set(0.61);
      m_color = 0.61;
    }else{
      m_blinkin.set(0.67);
      m_color = 0.67;
    }
  }

  // Sets the robot to the alliance color but blinking
  public void setColorAuton(Alliance color){
    if(color == Alliance.Blue){
      m_blinkin.set(-0.09);
      m_color = -0.09;
    }else if(color == Alliance.Red){
      m_blinkin.set(-0.11);
      m_color = -0.11;
    }else{
      m_blinkin.set(-0.07);
      m_color = -0.07;
    }
  }
  @Override
  public void periodic() {
    m_blinkin.set(m_color);
  }
}
