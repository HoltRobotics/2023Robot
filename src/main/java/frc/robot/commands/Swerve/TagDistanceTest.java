// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class TagDistanceTest extends PIDCommand {
  private final Limelight m_light;

  /**
   * A command that sets the robot to the right distance from the AprilTags.
   * @param swerve The Swerve Subsystem
   * @param light The Limelight Subsystem
   */
  public TagDistanceTest(Swerve swerve, Limelight light) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0), // Makes the PID controller
        // This should return the measurement
        () -> light.getDistance(), // Gives the PID controller the distance of the robot as the input.
        // This should return the setpoint (can also be a constant)
        () -> SwerveConstants.kDistanceFromTagMeters, // This is the setpoint of the PID controller, where we want the robot's distance to be.
        // This uses the output
        output -> {
          // Use the output here
          swerve.drive(new Translation2d(0, output), 0, false, false); // Takes the output of the PID controller and sends it to the drivetrain.
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_light = light; // Passes the subsystem to the rest of the command.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_light.getDistance() > 2) { // Checks to see if the robot is more than 2 meters from the target.
      return true; // If it is, ends the command.
    } else {
      return false; // If it is not, keep the command running.
    }
  }
}
