// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class AlignToTag extends Command {
	CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
	Vision limelight = Vision.getInstance();
	Pose2d pose;
	RobotCentricFacingAngle rot = new RobotCentricFacingAngle()
	.withHeadingPID(SwerveConstants.rotateKp, SwerveConstants.rotateKi, SwerveConstants.rotateKd);

	TrapezoidProfile ctrl = new TrapezoidProfile(new Constraints(2, 4));
	State endState;

	public AlignToTag() {
		addRequirements(drivetrain, limelight);
	}
	
	@Override
	public void initialize() {
		var llpos = LimelightHelpers.getTargetPose_RobotSpace(limelight.getCameraName());
		SmartDashboard.putNumberArray("alignToTag_targetpos_robotspace", llpos);
		// Pose2d tagPose = new Pose2d(llpos[2], llpos[0], new Rotation2d(llpos[4]));
		// endState = new State(0, 0);
	}
	
	@Override
	public void execute() {}
	
	@Override
	public void end(boolean interrupted) {}
	
	@Override
	public boolean isFinished() {
		return true;//Math.abs(limelight.getTX()) < VisionConstants.alignDeadband;
	}
}