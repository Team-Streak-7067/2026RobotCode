// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class Push extends InstantCommand {
	Intake intake = Intake.getInstance();
	
	public Push() {
		addRequirements(intake);
	}
	
	@Override
	public void initialize() {
		intake.push();
	}
}