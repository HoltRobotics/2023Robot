// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Swerve;

public class BalanceProfiled extends ProfiledPIDCommand {
  // private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");

  /** Creates a new BalanceProfiled. */
  public BalanceProfiled(Swerve swerve) {
    super(
      // The ProfiledPIDController used by the command
      new ProfiledPIDController(
        // The PID gains
        0.05, // 16
        0,
        0.07,
        // The motion profile constraints
        new TrapezoidProfile.Constraints(1, 5)
      ),
      // This should return the measurement
      () -> swerve.getPitch() + swerve.getRoll(),
      // This should return the goal (can also be a constant)
      () -> new TrapezoidProfile.State(0, 0),
      // This uses the output
      (output, setpoint) -> {
        // Use the output (and setpoint, if desired) here
      System.out.println("Slow: " + (swerve.getPitch() + swerve.getRoll()));
    // System.out.println(Math.abs(swerve.getPitch()) + Math.abs(swerve.getRoll()));
        if(Math.abs(swerve.getPitch()) + Math.abs(swerve.getRoll()) > 2) {
          swerve.drive(new Translation2d(-output, 0), 0, true, false);
        } else {
          swerve.drive(new Translation2d(), Math.toRadians(0), false, false);
        }
      }
    );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(swerve);
    // m_tab.add("Balance", getController());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
