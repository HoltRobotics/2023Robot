// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Swerve;

public class FastBalance extends PIDCommand {
  private final Swerve m_swerve;

  /** Creates a new FastBalance. */
  public FastBalance(Swerve swerve) {
    super(
        // The controller that the command will use
        new PIDController(0.10, 0, 0),
        // This should return the measurement
        () -> swerve.getPitch() + swerve.getRoll(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          swerve.drive(new Translation2d(-output, 0), 0, true, true);
        });
        m_swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    System.out.println("Fast: " + (swerve.getPitch() + swerve.getRoll()));
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_swerve.getPitch()) + Math.abs(m_swerve.getRoll()) < 10) {
      return true;
    } else {
      return false;
    }
  }
}
