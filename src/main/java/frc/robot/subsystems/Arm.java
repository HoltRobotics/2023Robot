// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.*;

public class Arm extends PIDSubsystem {
  // private final Elevator m_lift = new Elevator();
  // private final Pneumatics m_air = new Pneumatics();

  private final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);

  private final RelativeEncoder m_encoder;

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private final GenericEntry m_angleDisplay;

  private boolean m_inControl = false;

  /** Creates a new Arm. */
  public Arm() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD));
    m_encoder = m_armMotor.getAlternateEncoder(8192);
    m_encoder.setPositionConversionFactor(360);
    m_encoder.setPosition(0);
    m_armMotor.setInverted(false);
    m_tab.add("Arm", m_controller);
    m_angleDisplay = m_tab.add("Arm Angle", getAngle()).withWidget(BuiltInWidgets.kTextView).withPosition(3, 1).withSize(1, 1).getEntry();
    this.setAngle(0);
    this.enable();
    m_armMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_armMotor.setVoltage(output);
  }

  public void setAngle(double angle) {
    setSetpoint(angle);
  }

  public double getAngle() {
    return m_encoder.getPosition();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngle();
  }

  public void up() {
    m_armMotor.set(-0.75);
    m_inControl = true;
  }

  public void down() {
    m_armMotor.set(0.75);
    m_inControl = true;
  }

  public void stop() {
    m_armMotor.stopMotor();
    m_inControl = false;
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    super.periodic();
    m_angleDisplay.setDouble(getAngle());
    // if(ArmConstants.kArmLenght * Math.cos(getAngle()) + m_lift.getHeight() > Constants.kMaxRobotHeight && !m_inControl) {
    //   setAngle(Math.acos((Constants.kMaxRobotHeight - m_lift.getHeight()) / ArmConstants.kArmLenght));
    // }
    // if(ArmConstants.kArmLenght * Math.cos(getAngle()) + m_lift.getHeight() < Constants.kMinRobotHeight && !m_inControl) {
    //   if(m_air.valueToBool(m_air.getTiltState())) {
    //     setAngle(Math.acos(((Constants.kMaxRobotHeight + ArmConstants.kClawHeightOffset) - m_lift.getHeight()) / ArmConstants.kArmLenght));
    //   } else {
    //     setAngle(Math.acos((Constants.kMinRobotHeight - m_lift.getHeight()) / ArmConstants.kArmLenght));
    //   }
    // }
    if(getAngle() < 0) {
      setAngle(0);
    }
    if(getAngle() > ArmConstants.kMaxAngle) {
      setAngle(ArmConstants.kMaxAngle);
    }
  }
}
