// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class UpdateSetpoint extends InstantCommand {
	Shooter shooter = Shooter.getInstance();
	AngularVelocity setpoint;
	
	public UpdateSetpoint(AngularVelocity setpoint) {
		addRequirements(shooter);
		this.setpoint = setpoint;
	}

	@Override
	public void initialize() {
		shooter.updateSetpoint(setpoint);
	}
}