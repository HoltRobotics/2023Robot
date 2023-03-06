// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.Arm.SetAngle;
import frc.robot.commands.Elevator.SetHeight;
import frc.robot.commands.Pneumatics.ClawUp;
import frc.robot.commands.Pneumatics.OpenClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pneumatics;

public class GroundConePick extends ParallelCommandGroup {
  /**
   * Combo command that sets the arm, elevator, and claw to the right states to score on the low goal/pick up peices. Runs all the commands at once.
   * @param arm The Arm Subsystem
   * @param lift The Elevator Subsystem
   * @param air The Pneumatics Subsystem
   */
  public GroundConePick(Arm arm, Elevator lift, Pneumatics air) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetHeight(ElevatorConstants.kStage1Height, lift), // Sets the Elevator to the right height.
      new SetAngle(ArmConstants.kStage1Angle, arm), // Sets the arm to the right angle.
      new OpenClaw(air), // Makes sure the claw is open.
      new ClawUp(air) // Sets the claw level.
    );
  }
}
