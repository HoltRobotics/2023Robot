// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants;

public class Arm extends PIDSubsystem {
  private final CANSparkMax m_armMotor = new CANSparkMax(Constants.Arm.armMotorID, MotorType.kBrushless);

  private final RelativeEncoder m_encoder;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.Arm.kS, Constants.Arm.kV);

  /** Creates a new Arm. */
  public Arm() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD));
    m_encoder = m_armMotor.getAlternateEncoder(2048);
    m_encoder.setPositionConversionFactor(360);
    m_encoder.setPosition(0);
    this.setAngle(0);
    this.enable();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_armMotor.setVoltage(output + m_feedforward.calculate(setpoint));
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
}
