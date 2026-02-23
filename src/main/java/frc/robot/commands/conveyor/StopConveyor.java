// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;

public class StopConveyor extends InstantCommand {
	Conveyor conv = Conveyor.getInstance();
	
	public StopConveyor() {
		addRequirements(conv);
	}
	
	@Override
	public void initialize() {
		conv.stopAll();
	}
}