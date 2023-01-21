// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TestPath extends SequentialCommandGroup {
  /** Creates a new TestPath. */
  public TestPath(PathPlannerTrajectory traj, Swerve swerve) {
    addCommands(
      new InstantCommand(() -> {
        swerve.resetOdometry(traj.getInitialHolonomicPose());
      }),
      new PPSwerveControllerCommand(
        traj,
        swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(1.0, 0, 0),
        new PIDController(1.0, 0, 0),
        new PIDController(0.5, 0, 0),
        swerve::setModuleStates,
        swerve
      ),
      new InstantCommand(
        () -> {
          swerve.stopDrive();
        }
      )
    );
  }
}
