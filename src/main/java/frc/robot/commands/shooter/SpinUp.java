// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class SpinUp extends InstantCommand {
	Shooter shooter = Shooter.getInstance();
	Distance dist; 
	
	public SpinUp() {
		addRequirements(shooter);
	}
	
	@Override
	public void initialize() {
		shooter.spinUp(Robot.m_robotContainer.getDistanceToHub());
		shooter.setState(ShooterState.Shooting);
	}
}
