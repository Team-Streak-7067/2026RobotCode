// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class ReverseFeed extends InstantCommand {
	Intake intake = Intake.getInstance();
	Conveyor conv = Conveyor.getInstance();
	
	public ReverseFeed() {
		addRequirements(intake, conv);
	}
	
	@Override
	public void initialize() {
		intake.push();
		conv.reverse();
	}
}