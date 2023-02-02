// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Elevator extends PIDSubsystem {
  private final CANSparkMax m_liftmotor = new CANSparkMax(Constants.Elevator.elevatorMotorID, MotorType.kBrushless);

  private final Encoder m_encoder = new Encoder(Constants.Elevator.encoderPortA, Constants.Elevator.encoderPortB);

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private final GenericEntry m_height;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(Constants.Elevator.kS, Constants.Elevator.kV);
  /** Creates a new Elevator. */
  public Elevator() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD));
    m_encoder.setDistancePerPulse(Constants.Elevator.encoderDistancePerPulse);
    m_height = m_tab.add("Elevator Height", getElevatorHeight()).withWidget(BuiltInWidgets.kDial).withPosition(4, 1).withSize(1, 1).getEntry();
    this.setHeight(0);
    this.enable();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_liftmotor.setVoltage(output + m_feedforward.calculate(setpoint));
  }

  public void setHeight(double height) {
    setSetpoint(height);
  }

  public double getElevatorHeight() {
    return m_encoder.getDistance();
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getElevatorHeight();
  }

  @Override
  public void periodic() {
    super.periodic();
    m_height.setDouble(getElevatorHeight());
  }
}
