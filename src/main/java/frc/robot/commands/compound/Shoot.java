// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.conveyor.RunConveyor;
import frc.robot.commands.shooter.SpinUp;

public class Shoot extends ParallelCommandGroup {
	public Shoot() {
		addCommands(
			new SpinUp(),			
			new RunConveyor()
		);
	}
}