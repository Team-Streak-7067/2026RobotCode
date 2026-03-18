// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.RobotContainer;

public class Vision extends SubsystemBase {
	static Vision instance = null;
	static double counter = 0;

	final String name;
	PoseEstimate mt;
	CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;

	Vision(String name) {
		this.name = name;
		LimelightHelpers.setCropWindow(name, -1, 1, -0.7, 1);
	}
	
	@Override
	public void periodic() {
		LimelightHelpers.SetRobotOrientation(name, drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
		addVisionMeasurements();
	}

	public Pose2d getPose() {
		return mt.pose;
	}

	public Time getPoseTimestamp() {
		return Seconds.of(mt.timestampSeconds);
	}

	void addVisionMeasurements() {
		mt = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

		if (mt == null) {
			System.out.println("MT is null, skipping");
			return;
		}
		
		SmartDashboard.putNumber("FTID-" + name, getTID());
		try {
			SmartDashboard.putNumber("mt tag count", mt.tagCount);
			SmartDashboard.putNumber("Dist-" + name, mt.avgTagDist);
		} catch (Exception e) {}

		if (mt.tagCount == 0) {
			return;
		}
		
		boolean[] conds = {
			// if spinning fast
			drivetrain.getPigeon2().getAngularVelocityZDevice().asSupplier().get().abs(RotationsPerSecond) > SwerveConstants.maxAngularVelocity.abs(RotationsPerSecond),
			// if measurement pose is outside of field
			!FieldConstants.field.contains(mt.pose.getTranslation()),
		};
		
		for (int i = 0; i < conds.length; i++) {
			if (conds[i]) {
				return;
			}
		}

		double trust = .25;
		trust /= mt.tagCount;
		trust *= mt.avgTagDist / 1.5;

		// double dirTrust = Math.pow(Math.E, mt.avgTagDist * mt.avgTagDist);

		// latency compensation
		Time latency = Milliseconds.of(LimelightHelpers.getLatency_Pipeline(name) + LimelightHelpers.getLatency_Capture(name));
		ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());
		
		Pose2d compensatedPose = mt.pose.exp(
			new Twist2d(
				-fieldSpeeds.vxMetersPerSecond * latency.in(Seconds),
				-fieldSpeeds.vyMetersPerSecond * latency.in(Seconds),
				-fieldSpeeds.omegaRadiansPerSecond * latency.in(Seconds)
			)
		);

		// copied shamelessly from limelight docs and slightly adjusted (megatag localization)
		// drivetrain.addVisionMeasurement(compensatedPose, mt.timestampSeconds, VecBuilder.fill(trust, trust, dirTrust));
		drivetrain.addVisionMeasurement(compensatedPose, mt.timestampSeconds, VecBuilder.fill(trust, trust, Double.MAX_VALUE));

		SmartDashboard.putNumber(name + "trust", trust);
	}

	public static Vision getInstance() {
		if (instance == null) {
			instance = new Vision(VisionConstants.llName);
		}
		return instance;
	}

	public boolean getTV() {
		return LimelightHelpers.getTV(name);
	}

	public double getTX() {
		return getTV() ? LimelightHelpers.getTX(name) : -1;
	}

	public double getTY() {
		return getTV() ? LimelightHelpers.getTY(name) : -1;
	}

	public int getTID() {
			return getTV() ? (int)LimelightHelpers.getFiducialID(name) : -1;
	}

	public Distance getDistanceToTag() {
		if (mt == null) return Meters.of(0);

		return Meters.of(mt.avgTagDist);
	}

	public String getCameraName() {
		return name;
	}
}