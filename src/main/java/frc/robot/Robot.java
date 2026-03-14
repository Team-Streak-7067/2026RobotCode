// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.TSLib.leds.LedStrip;
import frc.TSLib.leds.LedStrip.LedStatus;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
    final LedStrip leds = RobotContainer.leds; 
	final Field2d field = new Field2d();
	CommandScheduler scheduler = CommandScheduler.getInstance();
	
	public static final RobotContainer m_robotContainer = new RobotContainer();
	static final CommandSwerveDrivetrain drivetrain = RobotContainer.drivetrain;
	
	public Robot() {
		SmartDashboard.putData(field);
		var x = FieldConstants.getHubPos();
		Pose2d hubPos = new Pose2d(x.getX(), x.getY(), Rotation2d.kZero);

		field.getObject("hub").setPose(hubPos);
	}

	@Override
	public void robotPeriodic() {
		scheduler.run();
		field.setRobotPose(drivetrain.getState().Pose);

		// FIXME remove later
		SmartDashboard.putNumber("dist to hub", m_robotContainer.getDistanceToHub().in(Meters));
	}
	
	@Override
	public void disabledInit() {
		leds.setStatus(LedStatus.Idle);
	}
	
	@Override
	public void disabledPeriodic() {}
	
	@Override
	public void disabledExit() {
		leds.setStatus(LedStatus.Off);
	}
	
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		
		if (m_autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(m_autonomousCommand);
		}
	}
	
	@Override
	public void autonomousPeriodic() {}
	
	@Override
	public void autonomousExit() {}
	
	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}
	
	@Override
	public void teleopPeriodic() {}
	
	@Override
	public void teleopExit() {}
	
	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}
	
	@Override
	public void testPeriodic() {}
	
	@Override
	public void testExit() {}
}