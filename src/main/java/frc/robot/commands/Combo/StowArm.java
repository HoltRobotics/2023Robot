// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combo;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetAngle;
import frc.robot.commands.Elevator.SetHeight;
import frc.robot.commands.Pneumatics.ClawDown;
import frc.robot.commands.Pneumatics.CloseClaw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pneumatics;

public class StowArm extends ParallelCommandGroup {
  /** Creates a new StowArm. */
  public StowArm(Arm arm, Elevator lift, Pneumatics air) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetHeight(Constants.Elevator.kStowHeight, lift),
      new SetAngle(Constants.Arm.kStowAngle, arm),
      new CloseClaw(air),
      new ClawDown(air)
    );
  }
}
