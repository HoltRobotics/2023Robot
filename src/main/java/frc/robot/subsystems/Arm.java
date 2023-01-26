// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import frc.robot.Constants;

public class Arm extends PIDSubsystem {
  private final CANSparkMax m_armMotor = new CANSparkMax(Constants.Arm.armMotorID, MotorType.kBrushless);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.Arm.kS, Constants.Arm.kV);

  /** Creates a new Arm. */
  public Arm() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.Arm.kP, Constants.Arm.kI, Constants.Arm.kD));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
