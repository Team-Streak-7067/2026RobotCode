// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.TSLib.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.TSLib.leds.LedStrip.LedStatus;
import frc.robot.RobotContainer;

public class SetLedState extends InstantCommand {
	LedStrip leds = RobotContainer.leds;
	LedStatus status;
	
	public SetLedState(LedStatus status) {
		addRequirements(leds);
		this.status = status;
	}
	
	@Override
	public void initialize() {
		leds.setStatus(status);
	}
}