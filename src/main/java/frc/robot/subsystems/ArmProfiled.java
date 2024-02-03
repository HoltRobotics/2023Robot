// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmProfiled extends ProfiledPIDSubsystem {
  private final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless);
  private final RelativeEncoder m_encoder;

  // private final ArmFeedforward m_feedforward = new ArmFeedforward(0, 0, 0, 0);

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  private final GenericEntry m_angleDisplay;
  // private final GenericEntry m_voltageDisplay;

  public boolean m_inMotion = false;
  private double m_setPoint = 0;

  /** Creates a new ArmProfiled. */
  public ArmProfiled() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
          2.5,
          0,
          0,
          // The motion profile constraints
          new TrapezoidProfile.Constraints(120, 90) // 90 90
        )
    );
    // m_encoder = m_armMotor.getAlternateEncoder(8192);
    m_encoder = m_armMotor.getEncoder();
    m_encoder.setPositionConversionFactor((44/12 * 125)/360);
    m_encoder.setPosition(0);
    m_angleDisplay = m_tab.add("Arm Angle", getAngle()).getEntry();
    // m_voltageDisplay = m_tab.add("Arm Volts", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
    // m_tab.add("Arm", m_controller);
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

  /**
   * Sets the PID setpoint.
   * @param angle The desired angle of the arm.
   */
  public void setAngle(double angle) {
    m_setPoint = angle;
    setGoal(angle); // Sets the PID setpoint to the given input.
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngle();
  }

  /**
   * Method for forcing the arm to move up.
   */
  public void up() {
    m_armMotor.set(-0.25); // Sets the speed of the motor to -3/4.
    // m_inControl = true;
  }

  /**
   * Method for forcing the arm to move down.
   */
  public void down() {
    m_armMotor.set(0.25); // Sets the speed of the motor to 3/4.
    // m_inControl = true;
  }

  public boolean getInMotion() {
    return m_inMotion;
  }

  public void setInMotion(boolean inMotion) {
    m_inMotion = inMotion;
  }


  public void setSpeed(double speed) {
    m_armMotor.set(speed);
  }

  @Override
  public void periodic() {
    super.periodic();
    m_angleDisplay.setDouble(getAngle());
    // m_voltageDisplay.setDouble(m_armMotor.get() * m_armMotor.getBusVoltage());
    if(getAngle() < -5) { // Checks to see if the arm is past the min limit.
      setAngle(0); // If it is set the PID to 0.
    }
    if(getAngle() > ArmConstants.kMaxAngle) { // Checks to see if the arm is past the max limit.
      setAngle(ArmConstants.kMaxAngle); // If is is set the PID to 180.
    }
    if(getAngle() > m_setPoint - 0.1 && getAngle() < m_setPoint + 0.1) {
      m_inMotion = false;
    }
  }
}
