// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorProfiled extends ProfiledPIDSubsystem {
  private final CANSparkMax m_liftmotor = new CANSparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless); // Makes the motor object.

  private final RelativeEncoder m_encoder; // Makes the encoder object.

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main"); // Gets the Shuffleboard tab.
  private final GenericEntry m_heightDisplay; // Makes the encoder display.

  public boolean m_inMotion = false;
  private double m_setPoint = 0;

  /** Creates a new ElevatorProfiled. */
  public ElevatorProfiled() {
    super(
      // The ProfiledPIDController used by the subsystem
      new ProfiledPIDController(
        200, //200
        0,
        0,
        // The motion profile constraints
        new TrapezoidProfile.Constraints(0.8, 1)
      )
    );
    m_encoder = m_liftmotor.getEncoder(); // Gets the encoder built into the motor.
    m_encoder.setPositionConversionFactor(Units.inchesToMeters(Math.PI * 1.44) / 16); // Sets the encoder units to meters traveled by the elevator.
    m_encoder.setPosition(0); // Sets the encoder to 0 meters.
    m_tab.add("Elevator", m_controller); // Adds the PID controller to the suffleboard for tuning.
    m_liftmotor.setInverted(false);
    m_liftmotor.setSmartCurrentLimit(30);
    m_heightDisplay = m_tab.add("Elevator Height", getHeight()).withWidget(BuiltInWidgets.kTextView).withPosition(4, 1).withSize(1, 1).getEntry(); // Adds the elevator height to the shuffleboard tab.
    this.setHeight(0); // Sets the PID controller to 0 meters.
    this.enable(); // Turns on the PID contorller.
    m_liftmotor.setIdleMode(IdleMode.kBrake); // Default motor behavior is break mode.
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    m_liftmotor.setVoltage(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getHeight(); // Gets the current encoder position.
  }

  /**
   * Sets the PID setpoint.
   * @param height The desired height of the elevator in meters.
   */
  public void setHeight(double height) {
    m_setPoint = height;
    setGoal(height); // Sets the PID setpoint to the given input.
    // System.out.println(height);
    // setGoal(new TrapezoidProfile.State(height, 0));
  }

  /**
   * Gets the current height of the elevator.
   * @return Height of the elevator in meters.
   */
  public double getHeight() {
    // System.out.println(m_encoder.getPosition());
    return m_encoder.getPosition(); // Gets the current encoder position.
  }

  /**
   * Method for forcing the elevator up.
   */
  public void up() {
    m_liftmotor.set(.5); // Sets the speed of the motor to half.
    // m_inControl = true;
  }

  /**
   * Method for forceing the elevator down.
   */

  public void down() {
    m_liftmotor.set(-0.5); // Sets the speed of the motor to negative half.
    // m_inControl = true;
  }

  
  /**
   * Method for stopping the motor.
   */
  public void stop() {
    m_liftmotor.stopMotor(); // Stops the motor.
    // m_inControl = false;
  }

  /**
   * Resets the elevator encoder back to 0.
   */
  public void resetEncoder() {
    m_encoder.setPosition(0); // Sets the encoder to 0 meters.
  }

  @Override
  public void periodic() {
    super.periodic(); // Runs the PID contorller.
    m_heightDisplay.setDouble(getHeight()); // Updates the height on shuffleboard.
    // System.out.println("Lift PID: " + isEnabled());
    if(getHeight() > ElevatorConstants.kMaxHeight) { // Checks to see if the elevator is past the max limit.
      setHeight(ElevatorConstants.kMaxHeight); // If it is, set the PID to the max height.
    }
    if(getHeight() < -0.05) { // Checks to see if the arm is past the min limit.
      setHeight(0); // If is is, set the PID to the min height.
    }
    if(getHeight() > m_setPoint - 0.05 && getHeight() < m_setPoint + 0.05) {
      m_inMotion = false;
    }
  }
}
