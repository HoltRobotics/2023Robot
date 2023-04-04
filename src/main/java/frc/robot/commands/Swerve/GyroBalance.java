// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Swerve;

public class GyroBalance extends PIDCommand {
  // private final Swerve m_swerve;
  // private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
  /**
   * Command that auto balances the robot on the docking station.
   * @param swerve The Swerve Subsystem
   */
  public GyroBalance(Swerve swerve) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0), // Makes the PID controller.
        // This should return the measurement
        () -> swerve.getPitch(), // Gives the PID controller the pitch of the robot as the input.
        // This should return the setpoint (can also be a constant)
        () -> 0, // This is the setpoint of the PID controller, where we want the robot's pitch to be.
        // This uses the output
        output -> {
          // Use the output here
          swerve.drive(new Translation2d(-output, 0), 0, false, false); // Takes the output of the PID controller and sends it to the drivetrain.
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(swerve); // Stops all other commands using the Swerve subsystem.
    // m_tab.add("Balance", getController());
    // m_swerve = swerve;
    m_controller.setTolerance(5);
    // m_tab.add("Pitch", m_swerve.getPitch());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    // System.out.println("Pitch: " + m_swerve.getPitch());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
