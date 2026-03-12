// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.compound;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentricFacingAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
	
	public AlignToTag() {
		addRequirements(drivetrain, limelight);
	}
	
	@Override
	public void initialize() {
		double[] llArr = LimelightHelpers.getBotPose_TargetSpace(limelight.getCameraName());
		// what i get out of limelight should be x,y, th of robot in target space
		pose = new Pose2d(llArr[0], llArr[1], new Rotation2d(llArr[2]));
		drivetrain.applyRequest(()->rot.withTargetDirection(pose.getRotation().unaryMinus()));
		// TODO continue
	}
	
	@Override
	public void execute() {}
	
	@Override
	public void end(boolean interrupted) {}
	
	@Override
	public boolean isFinished() {
		return false;
	}
}