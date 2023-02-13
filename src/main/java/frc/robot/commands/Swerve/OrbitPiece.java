// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class OrbitPiece extends CommandBase {
  private final Swerve m_swerve;
  private final Elevator m_lift;
  private final Arm m_arm;
  private SwerveDriveKinematics m_kinematics;
  
  /** Creates a new OrbitPiece. */
  public OrbitPiece(Swerve swerve, Elevator lift, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve;
    m_lift = lift;
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: enter the right formulas
    // For now it just changes the "center" of the robot to the middle front of the robot
    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(-Units.inchesToMeters(24.5 / 2), -1),
      new Translation2d(Units.inchesToMeters(24.5 / 2), -1),
      new Translation2d(-Units.inchesToMeters(24.5 / 2), -Units.inchesToMeters(24.5) + 1),
      new Translation2d(Units.inchesToMeters(24.5 / 2), -Units.inchesToMeters(24.5) + 1)
    );
    m_lift.getElevatorHeight();
    m_arm.getAngle();
    m_swerve.setKinematic(m_kinematics);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.resetKinematic();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
