// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import frc.robot.Constants;
import frc.robot.Constants.*;

public class Elevator extends PIDSubsystem {
  // private final Arm m_arm = new Arm();
  // private final Pneumatics m_air = new Pneumatics();
  
  private final CANSparkMax m_liftmotor = new CANSparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless); // Makes the motor object.

  private final RelativeEncoder m_encoder; // Makes the encoder object.

  private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main"); // Gets the Shuffleboard tab.
  private final GenericEntry m_heightDisplay; // Makes the encoder display.

  // private boolean m_inControl = false;

  /** Creates a new Elevator. */
  public Elevator() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD)); // Makes the PID controller.
    m_encoder = m_liftmotor.getEncoder(); // Gets the encoder built into the motor.
    m_encoder.setPositionConversionFactor(Units.inchesToMeters(Math.PI * 1.44) / 16); // Sets the encoder units to meters traveled by the elevator.
    m_encoder.setPosition(0); // Sets the encoder to 0 meters.
    // m_tab.add("Elevator", m_controller); // Adds the PID controller to the suffleboard for tuning.
    m_liftmotor.setInverted(false); // Inverts the motor.
    m_liftmotor.setSmartCurrentLimit(30);
    m_heightDisplay = m_tab.add("Elevator Height", getHeight()).withWidget(BuiltInWidgets.kTextView).withPosition(4, 1).withSize(1, 1).getEntry(); // Adds the elevator height to the shuffleboard tab.
    this.setHeight(0); // Sets the PID controller to 0 meters.
    this.enable(); // Turns on the PID contorller.
    m_liftmotor.setIdleMode(IdleMode.kBrake); // Default motor behavior is break mode.
  }

  /**
   * Output function for the PID controller. Sets the elevator voltage to the output of the PID controller.
   */
  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_liftmotor.setVoltage(output); // Sets the motor to the given voltage. Will adjust for voltage sag.
  }

  /**
   * Sets the PID setpoint.
   * @param height The desired height of the elevator in meters.
   */
  public void setHeight(double height) {
    setSetpoint(height); // Sets the PID setpoint to the given input.
  }

  /**
   * Gets the current height of the elevator.
   * @return Height of the elevator in meters.
   */
  public double getHeight() {
    return m_encoder.getPosition(); // Gets the current encoder position.
  }

  /**
   * Input function of the PID controller. Get the current height of the elevator.
   */
  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getHeight(); // Gets the current encoder position.
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

  /**
   * This method runs every 20ms while the robot is on.
   */
  @Override
  public void periodic() {
    super.periodic(); // Runs the PID contorller.
    m_heightDisplay.setDouble(getHeight()); // Updates the height on shuffleboard.
    // System.out.println(getSetpoint());
    // This block of code needs to be worked on. Might not work. Might need to combine Arm, Elevator, and Pneumatics together to make work.
    // if(ArmConstants.kArmLenght * Math.cos(m_arm.getAngle()) + getHeight() > Constants.kMaxRobotHeight && !m_inControl) {
    //   setHeight(Constants.kMaxRobotHeight - (ArmConstants.kArmLenght * Math.cos(m_arm.getAngle())));
    // }
    // if(ArmConstants.kArmLenght * Math.cos(m_arm.getAngle()) + getHeight() < Constants.kMinRobotHeight && !m_inControl) {
    //   if(m_air.valueToBool(m_air.getTiltState())) {
    //     setHeight((Constants.kMinRobotHeight + ArmConstants.kClawHeightOffset) - (ArmConstants.kArmLenght * Math.cos(m_arm.getAngle())));
    //   } else {
    //     setHeight(Constants.kMinRobotHeight - (ArmConstants.kArmLenght * Math.cos(m_arm.getAngle())));
    //   }
    // }
    if(getHeight() > ElevatorConstants.kMaxHeight) { // Checks to see if the elevator is past the max limit.
      setHeight(ElevatorConstants.kMaxHeight); // If it is, set the PID to the max height.
    }
    if(getHeight() < -0.05) { // Checks to see if the arm is past the min limit.
      setHeight(0); // If is is, set the PID to the min height.
    }
  }
}
