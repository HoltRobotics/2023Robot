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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends PIDSubsystem {
  private final CANSparkMax m_liftmotor = new CANSparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

  private final RelativeEncoder m_encoder;

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private final GenericEntry m_height;

  /** Creates a new Elevator. */
  public Elevator() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD));
    m_encoder = m_liftmotor.getAlternateEncoder(8192);
    m_encoder.setPositionConversionFactor(0.01); //TODO: put in right factor
    m_encoder.setPosition(0);
    m_height = m_tab.add("Elevator Height", getHeight()).withWidget(BuiltInWidgets.kTextView).withPosition(4, 1).withSize(1, 1).getEntry();
    this.setHeight(0);
    this.enable();
    m_liftmotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_liftmotor.setVoltage(output);
  }

  public void setHeight(double height) {
    setSetpoint(height);
  }

  public double getHeight() {
    return m_encoder.getPosition();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getHeight();
  }

  public void up() {
    m_liftmotor.set(.5);
  }

  public void down() {
    m_liftmotor.set(-0.5);
  }

  public void stop() {
    m_liftmotor.stopMotor();
  }

  @Override
  public void periodic() {
    super.periodic();
    m_height.setDouble(getHeight());
    if(getHeight() > 1) {
      setHeight(0);
    }
    if(getHeight() < 0) {
      setHeight(0);
    }
  }
}
