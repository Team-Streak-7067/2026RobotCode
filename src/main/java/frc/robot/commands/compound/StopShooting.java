// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.conveyor.StopConveyor;
import frc.robot.commands.shooter.StopShooter;

public class StopShooting extends SequentialCommandGroup {
	public StopShooting() {
		addCommands(
			new StopConveyor(),
			new StopShooter()
		);
	}
}