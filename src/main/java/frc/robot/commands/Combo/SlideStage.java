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
import frc.robot.subsystems.ArmProfiled;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pneumatics;

public class SlideStage extends ParallelCommandGroup {
  /**
   * Combo command that sets the arm, elevator, and claw to the right states to grab from double substation. Runs all the commands at once.
   * @param arm The Arm Subsystem
   * @param lift The Elevator Subsystem
   * @param air The Pneumatics Subsystem
   */
  public SlideStage(ArmProfiled arm, Elevator lift, Pneumatics air) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetHeight(ElevatorConstants.kSlideStageHeight, lift), // Sets the Elevator to the right height.
      new SetAngle(ArmConstants.kHumanStageAngle, arm), // Sets the arm to the right angle.
      new OpenClaw(air), // Makes sure the claw is closed.
      new ClawUp(air) // Sets the claw level.
    );
  }
}
