// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static java.lang.Math.cos;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
	static Shooter instance = new Shooter();
	static CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
	
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
		cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		cfg.Slot0 = ShooterConstants.S0C;
		cfg.Feedback.SensorToMechanismRatio = 1/ShooterConstants.ratio;
		leader.getConfigurator().apply(cfg);
		leader.setControl(ctrl);

		ctrl.Slot = 0;

		follower.setControl(new Follower(ShooterConstants.leaderCANID, MotorAlignmentValue.Opposed));
	}

	void populateMap() {
		if (ShooterConstants.speedMapData.length == 0) return;

		for (double[] datum: ShooterConstants.speedMapData) {
			speedMap.put(datum[0], datum[1]);
		}
	}

	public void updateSetpoint(AngularVelocity setpoint) {
		leader.setControl(ctrl.withVelocity(setpoint));
	}

	public void stop() {
		leader.stopMotor();
	}

	public AngularVelocity calcSpeed(Distance dist) {
		return calcSpeedRegression(dist);
	}

	Translation2d getShotVector_eeshwark(Translation2d goalPos) {
		Translation2d robot = RobotContainer.getRobotPose().getTranslation();
		Translation2d targetVec = goalPos.minus(robot);
		
		double dist = targetVec.getNorm();
		double idealSpeed = calcSpeed(Meters.of(dist)).in(RPM) * cos(ShooterConstants.shooterAngle.in(Radians));

		targetVec = targetVec.div(dist).times(idealSpeed);

		var speeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());
		Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond); 
		
		Translation2d shotVec = targetVec.minus(robotVel);

		// shotVec.getAngle() -> angle of robot relative to field
		// shotVec.getNorm() -> *VELOCITY* the balls needs to go
		// v = 0.5 * w * r
		// (may need to remove the *0.5)
		// v = ball velocirt
		// w = wheel angular velocty
		// r = shooter wheel radius
		shotVec.div(0.5 * ShooterConstants.wheelRadius.in(Meters)); // remove half if speed is incorrect
		// shotVec.getNorm() now returns shooter rpm

		return shotVec;
	}

	Translation2d getShotVector_mine(Translation2d goalPos) {
        Translation2d robot = RobotContainer.getRobotPose().getTranslation();
		var speeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());
		Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond); 
		Translation2d target = goalPos.minus(robot).minus(robotVel);

		return target;
	}
	
	public Translation2d getShotVector(Translation2d goalPos) {
		return getShotVector_mine(goalPos);
		// return getShotVector_int(goalPos);
	}

	// benched because regression was better
	AngularVelocity calcSpeedLUT(Distance dist) {
		return RotationsPerSecond.of(speedMap.get(dist.in(Meters)));
	}

	AngularVelocity calcSpeedRegression(Distance dist) {
		double x = dist.in(Meters);
		return RotationsPerSecond.of(106.07954/(1+Math.pow(Math.E, -(0.54042*x - 0.617275))));
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

	public void stopAll() {
		try {
			getCurrentCommand().cancel();
		} catch (Exception e) {}
		stop();
	}
}