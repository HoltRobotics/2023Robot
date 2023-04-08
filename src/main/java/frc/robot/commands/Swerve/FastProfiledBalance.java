// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Swerve;

public class FastProfiledBalance extends ProfiledPIDCommand {
  private final Swerve m_drive;

  /** Creates a new FastProfiledBalance. */
  public FastProfiledBalance(Swerve drive) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            1,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1, 1)),
        // This should return the measurement
        () -> drive.getPitch() + drive.getRoll(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drive.drive(new Translation2d(-output, 0), 0, false, true);
          System.out.println("Pro Speed: " + -output);
        });
        m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void execute() {
    System.out.println("Pro: " + m_drive.getPitch());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_drive.getPitch()) + Math.abs(m_drive.getRoll()) < 10) {
      return true;
    } else {
      return false;
    }
  }
}
