// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShooterState;

public class SpinUp extends InstantCommand {
	Shooter shooter = Shooter.getInstance();
	Distance dist; 
	AngularVelocity rate;
	Translation2d goalPos;
	
	public SpinUp(Translation2d goalPos) {
		this.goalPos = goalPos;
	}

	public SpinUp(AngularVelocity rate) {
		this.rate = rate;
	}

	public SpinUp(Distance dist) {
		this.dist = dist;
	}
	
	@Override
	public void initialize() {
		if (rate != null) {
			shooter.updateSetpoint(rate);
		} else {
			if (dist == null) {
				dist = Meters.of(shooter.getShotVector(goalPos).getNorm());
			}
			shooter.spinUp(dist);
		}
		shooter.setState(ShooterState.Shooting);
	}
}
