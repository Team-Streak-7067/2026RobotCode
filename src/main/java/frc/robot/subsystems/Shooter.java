// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
	static Shooter instance = new Shooter();
	
	TalonFX leader = new TalonFX(ShooterConstants.leaderCANID);
	TalonFX follower = new TalonFX(ShooterConstants.followerCANID);

	VelocityVoltage ctrl = new VelocityVoltage(0);
	InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap(); // dist (m) => speed (rps)

	ShooterState state = ShooterState.Off;
	
	public static Shooter getInstance() {
		return instance;
	}
	
	Shooter() {
		configMotors();
		populateMap();
	}
	
	@Override
	public void periodic() {
	}

	void configMotors() {
		follower.setNeutralMode(NeutralModeValue.Coast);
		
		TalonFXConfiguration cfg = new TalonFXConfiguration();
		cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		cfg.Slot0 = ShooterConstants.S0C;
		cfg.Feedback.SensorToMechanismRatio = 1/ShooterConstants.ratio;
		leader.getConfigurator().apply(cfg);
		leader.setControl(ctrl);

		ctrl.Slot = 0;

		follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));
	}

	void populateMap() {
		if (ShooterConstants.speedMapData.length == 0) return;

		for (double[] datum: ShooterConstants.speedMapData) {
			speedMap.put(datum[0], datum[1]);
		}
	}

	public void updateSetpoint(AngularVelocity setpoint) {
		ctrl.Velocity = setpoint.in(RotationsPerSecond);
	}

	public void stop() {
		leader.stopMotor();
	}

	AngularVelocity calcSpeed(Distance dist) {
		return RotationsPerSecond.of(speedMap.get(dist.in(Meters)));
	}

	public void spinUp(Distance distance) {
		updateSetpoint(calcSpeed(distance));
	}

	public ShooterState getState() {
		return state;
	}

	public void setState(ShooterState state) {
		this.state = state;
	}

	public enum ShooterState {
		Off,
		Idle,
		Shooting
	}
}