// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class SpinUp extends InstantCommand {
	Shooter shooter = Shooter.getInstance();
	Distance dist; // TODO switch out with proper distance when swerve and odometry is implemented
	
	public SpinUp() {
		addRequirements(shooter);
	}
	
	@Override
	public void initialize() {
		shooter.spinUp(dist);
		shooter.setState(ShooterState.Shooting);
	}
}
