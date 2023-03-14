// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmProfiled extends ProfiledPIDSubsystem {
  private final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
  private final RelativeEncoder m_encoder;

  private final ArmFeedforward m_feedforward = new ArmFeedforward(0, 0, 0, 0);

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private final GenericEntry m_angleDisplay;

  /** Creates a new ArmProfiled. */
  public ArmProfiled() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
          4.85,
          0,
          0.05,
          // The motion profile constraints
          new TrapezoidProfile.Constraints(60, 90)
        )
    );
    // m_encoder = m_armMotor.getAlternateEncoder(8192);
    m_encoder = m_armMotor.getEncoder();
    m_encoder.setPositionConversionFactor((44/12 * 125)/360);
    m_encoder.setPosition(0);
    m_angleDisplay = m_tab.add("Arm Angle", getAngle()).getEntry();
    m_tab.add("Arm", m_controller);
    this.setGoal(0);
    this.enable();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    // m_armMotor.setVoltage(output + m_feedforward.calculate(setpoint.position, setpoint.velocity));
    m_armMotor.setVoltage(output);
  }

  /**
   * Gets the current angle of the arm.
   * @return Angle of the arm.
   */
  public double getAngle() {
    return m_encoder.getPosition(); // Returns the angle of the arm.
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngle();
  }

  @Override
  public void periodic() {
    super.periodic();
    m_angleDisplay.setDouble(getAngle());
  }
}
