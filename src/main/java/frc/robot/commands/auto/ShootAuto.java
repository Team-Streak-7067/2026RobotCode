// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.compound.Shoot;
import frc.robot.commands.compound.StopShooting;

public class ShootAuto extends SequentialCommandGroup {
	public ShootAuto() {
		addCommands(
			new Shoot(RobotContainer.getShotNorm()),
			new WaitCommand(5),
			new StopShooting()
		);
	}
}