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
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import frc.robot.Constants;
import frc.robot.Constants.*;

public class Arm extends PIDSubsystem {
  // private final Elevator m_lift = new Elevator();
  // private final Pneumatics m_air = new Pneumatics();

  private final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.armMotorID, MotorType.kBrushless); // Makes the motor object.

  private final RelativeEncoder m_encoder; // Makes the encoder object.

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main"); // Gets the Shuffleboard tab.
  private final GenericEntry m_angleDisplay; // Makes the encoder display.

  // private boolean m_inControl = false;

  /** Creates a new Arm. */
  public Arm() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD)); // Makes the PID controller.
    m_encoder = m_armMotor.getAlternateEncoder(8192); // Points to the encoder and sets the CPR.
    m_encoder.setPositionConversionFactor(360); // Turns the units to degrees.
    m_encoder.setPosition(0); // Sets the encoder to 0 degrees.
    m_armMotor.setInverted(false); // Motor is not inverted.
    m_tab.add("Arm", m_controller); // Adds the PID controller to the shuffleboard for tuning.
    m_angleDisplay = m_tab.add("Arm Angle", getAngle()).withWidget(BuiltInWidgets.kTextView).withPosition(3, 1).withSize(1, 1).getEntry(); // Adds the arm angle to the shuffleboard tab.
    this.setAngle(0); // Sets the PID controller to 0 degrees.
    this.enable(); // Turns on the PID controller.
    m_armMotor.setIdleMode(IdleMode.kBrake); // Default motor behavior is brake mode.
  }

  /**
   * Output function for the PID controller. Sets the arm voltage to the output of the PID controller.
   */
  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_armMotor.setVoltage(output); // Sets the motor to the given voltage. Will adjust for voltage sag.
  }

  /**
   * Sets the PID setpoint.
   * @param angle The desired angle of the arm.
   */
  public void setAngle(double angle) {
    setSetpoint(angle); // Sets the PID setpoint to the given input.
  }

  /**
   * Gets the current angle of the arm.
   * @return Angle of the arm.
   */
  public double getAngle() {
    return m_encoder.getPosition(); // Returns the angle of the arm.
  }

  /**
   * Input function of the PID controller. Get the current angle of the arm.
   */
  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngle(); // Returns the angle of the arm.
  }

  /**
   * Method for forcing the arm to move up.
   */
  public void up() {
    m_armMotor.set(-0.75); // Sets the speed of the motor to -3/4.
    // m_inControl = true;
  }

  /**
   * Method for forcing the arm to move down.
   */
  public void down() {
    m_armMotor.set(0.75); // Sets the speed of the motor to 3/4.
    // m_inControl = true;
  }

  /**
   * Method for stopping the motor.
   */
  public void stop() {
    m_armMotor.stopMotor(); // Stops the motor.
    // m_inControl = false;
  }

  /**
   * Resets the arm encoder back to 0.
   */
  public void resetEncoder() {
    m_encoder.setPosition(0); // Sets the encoder to 0 degrees.
  }

  /**
   * This method runs every 20ms while the robot is on.
   */
  @Override
  public void periodic() {
    super.periodic(); // Runs the PID contorller.
    m_angleDisplay.setDouble(getAngle()); // Updates the angle on shuffleboard.
    // This block of code needs to be worked on. Might not work. Might need to combine Arm, Elevator, and Pneumatics together to make work.
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
    if(getAngle() < -1) { // Checks to see if the arm is past the min limit.
      setAngle(0); // If it is set the PID to 0.
    }
    if(getAngle() > ArmConstants.kMaxAngle) { // Checks to see if the arm is past the max limit.
      setAngle(ArmConstants.kMaxAngle); // If is is set the PID to 180.
    }
  }
}
