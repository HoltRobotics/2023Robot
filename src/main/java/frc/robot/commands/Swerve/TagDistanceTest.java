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

  /** Creates a new TagDistanceTest. */
  public TagDistanceTest(Swerve swerve, Limelight light) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0), //TODO: tune theses should be the same as the translation pid
        // This should return the measurement
        () -> light.getDistance(),
        // This should return the setpoint (can also be a constant)
        () -> SwerveConstants.kDistanceFromTagMeters,
        // This uses the output
        output -> {
          // Use the output here
          swerve.drive(new Translation2d(output, 0), 0, false, false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_light = light;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_light.getDistance() > 2) {
      return true;
    } else {
      return false;
    }
  }
}
