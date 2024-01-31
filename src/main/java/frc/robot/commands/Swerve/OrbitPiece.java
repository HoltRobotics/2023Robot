// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Swerve;

public class OrbitPiece extends Command {
  private final Swerve m_swerve; // The subsystem needed to control the Swerve.
  private final Arm m_arm; // The subsystem needed to control the arm.
  private SwerveDriveKinematics m_kinematics; // The kinematics we want to set to the wheels.
  
  /**
   * A Command that finds the point of our claw and sets that to our rotation point.
   * @param swerve The Swerve Subsystem
   * @param arm The Arm Subsystem
   */
  public OrbitPiece(Swerve swerve, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_swerve = swerve; // Passes the subsystem to the rest of the command.
    m_arm = arm; // Passes the subsystem to the rest of the command.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_kinematics = new SwerveDriveKinematics( // Sets the Kinematic to the point of our claw.
      new Translation2d(-((ArmConstants.kArmLenght * Math.sin(m_arm.getAngle()) + ArmConstants.kClawLenghtOffset) - Constants.ElevatorConstants.kFrontWheelsOffset), Constants.SwerveConstants.trackWidth / 2.0), // Front Left
      new Translation2d(-((ArmConstants.kArmLenght * Math.sin(m_arm.getAngle()) + ArmConstants.kClawLenghtOffset) - Constants.ElevatorConstants.kFrontWheelsOffset), -Constants.SwerveConstants.trackWidth / 2.0), // Front Right
      new Translation2d(-((ArmConstants.kArmLenght * Math.sin(m_arm.getAngle()) + ArmConstants.kClawLenghtOffset) + Constants.ElevatorConstants.kBackWheelsOffset), Constants.SwerveConstants.trackWidth / 2.0), // Back Left
      new Translation2d(-((ArmConstants.kArmLenght * Math.sin(m_arm.getAngle()) + ArmConstants.kClawLenghtOffset) + Constants.ElevatorConstants.kBackWheelsOffset), -Constants.SwerveConstants.trackWidth / 2.0) // Back Right
    );
    m_swerve.setKinematic(m_kinematics); // Sets the Kinematic to the wheels
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.resetKinematic(); // Puts the rotation point back to the center of our robot.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
