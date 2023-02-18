// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Swerve;

public class GyroBalance extends PIDCommand {
  /** Creates a new GyroBalance. */
  public GyroBalance(Swerve swerve) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0), //TODO: tune these, needs the robot done
        // This should return the measurement
        () -> swerve.getPitch(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          swerve.drive(new Translation2d(0, output), 0, false, false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(swerve);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
