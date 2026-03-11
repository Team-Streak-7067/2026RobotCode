// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.compound.Shoot;
import frc.robot.commands.compound.StopShooting;
import frc.robot.commands.conveyor.StopConveyor;
import frc.robot.commands.intake.Pull;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.intake.UpdateSetpoint;
import frc.robot.commands.shooter.Idle;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

	final SwerveRequest.ApplyRobotSpeeds driveRobot = new SwerveRequest.ApplyRobotSpeeds()
        .withDesaturateWheelSpeeds(false)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.Position);

    final SwerveRequest.FieldCentricFacingAngle face = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    	.withHeadingPID(SwerveConstants.rotateKp, SwerveConstants.rotateKi, SwerveConstants.rotateKd) // this is found through swerve rotation sysid
    ;

    private final Telemetry logger = new Telemetry(MaxSpeed);

	SendableChooser<Command> autoChooser;
	RobotConfig robotConfig;

    double slowMult = 1;

    private final CommandXboxController joystick = new CommandXboxController(0);

    public static final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    public static final Vision limelight = Vision.getInstance();
    public static final Intake intake = Intake.getInstance();
    public static final Conveyor conveyor = Conveyor.getInstance();
    public static final Shooter shooter = Shooter.getInstance();

	// BINDS for buttons
	final Trigger zeroHeadingButton = joystick.leftBumper().and(joystick.rightBumper());
	final Trigger slowButton = joystick.leftTrigger();
	final Trigger alignButton = joystick.b();
	final Trigger shootButton = joystick.rightTrigger();
	final Trigger intakeDownButton = joystick.povDown();
	final Trigger intakeUpButton = joystick.povUp();
	final Trigger reverseButton = joystick.povLeft();
	final Trigger intakeButton = joystick.rightBumper().and(zeroHeadingButton.negate());
	final Trigger confirmButton = joystick.povRight();

	public RobotContainer() {
		configureBindings();
        configPathplanner();
	}

	void configPathplanner() {
		try {
			robotConfig = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			e.printStackTrace();
		}
		
		AutoBuilder.configure(
			()->drivetrain.getState().Pose,
			drivetrain::resetPose,
			()->drivetrain.getState().Speeds,
			(speeds, ff)->drivetrain.setControl(driveRobot
				// .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                .withSpeeds(speeds)
				.withWheelForceFeedforwardsX(ff.robotRelativeForcesX())
				.withWheelForceFeedforwardsY(ff.robotRelativeForcesY())
			),
			new PPHolonomicDriveController(
				new PIDConstants(AutoConstants.driveKp, AutoConstants.driveKi, AutoConstants.driveKd),
				new PIDConstants(AutoConstants.steerKp, AutoConstants.steerKi, AutoConstants.steerKd)
			),
			robotConfig,
			()->DriverStation.getAlliance().orElse(Alliance.Blue) == DriverStation.Alliance.Red,
			drivetrain
		);

		NamedCommands.registerCommand("shooterIdle",new Idle());
		NamedCommands.registerCommand("shoot",new ShootAuto());
		NamedCommands.registerCommand("intakeDown", new UpdateSetpoint(IntakeConstants.openPos));
		NamedCommands.registerCommand("intakeUp", new UpdateSetpoint(IntakeConstants.resetPos));
		NamedCommands.registerCommand("intakeStart", new Pull());
		NamedCommands.registerCommand("intakeStop", new StopIntake());

		autoChooser = AutoBuilder.buildAutoChooser();
		Commands.runOnce(FollowPathCommand::warmupCommand);
		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

    Pose2d getRobotPose() {
		return drivetrain.getState().Pose;
	}

    Rotation2d getDirectionToHub() {
		Translation2d hub = FieldConstants.getHubPos();
		Translation2d robot = getRobotPose().getTranslation();

		Rotation2d angle = hub.minus(robot).getAngle();

        SmartDashboard.putNumber("angle to hub CCW+", angle.getDegrees());
        
		return angle;
	}

    public Distance getDistanceToHub() {
        Translation2d hub = FieldConstants.getHubPos();
        Translation2d robot = getRobotPose().getTranslation();

        return Meters.of(hub.minus(robot).getNorm());
    }

    boolean isInScoringZone() {
		Rectangle2d zone = FieldConstants.getScoringZone();
		Translation2d robot = getRobotPose().getTranslation();
		
		return zone.contains(robot);
	}

	boolean isInNeutralZone() {
		return FieldConstants.neutralZone.contains(getRobotPose().getTranslation());
	}

    void updateSlow(boolean slow) {
        slowMult = slow ? SwerveConstants.slowMult : 1;
    }
	
	private void configureBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * slowMult) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * slowMult) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * slowMult) // Drive counterclockwise with negative X (left)
            )
        );

        alignButton.whileTrue(
			drivetrain.applyRequest(()->face
				.withTargetDirection(getDirectionToHub())
                .withMaxAbsRotationalRate(MaxAngularRate / 1.5)
				.withVelocityX(-joystick.getLeftY() * MaxSpeed * slowMult) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed * slowMult) // Drive left with negative X (left)
			)
		);

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // add logger start/stop binds
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        zeroHeadingButton.onTrue(Commands.runOnce(drivetrain::seedFieldCentric));
        slowButton.onChange(Commands.runOnce(()->updateSlow(slowButton.getAsBoolean())));

        // joystick.a()
		// 	.onTrue(Commands.runOnce(intake::angleDown))
		// 	.onFalse(Commands.runOnce(intake::stopAngle));

        // joystick.y()
		// 	.onTrue(Commands.runOnce(intake::angleUp))
		// 	.onFalse(Commands.runOnce(intake::stopAngle));

        intakeButton.onTrue(new Pull()).onFalse(new StopIntake());

        intakeUpButton.onTrue(new UpdateSetpoint(IntakeConstants.openPos));
        intakeDownButton.onTrue(new UpdateSetpoint(IntakeConstants.resetPos));

        shootButton.onTrue(new Shoot()).onFalse(new StopShooting());

        reverseButton
			.onTrue(Commands.runOnce(conveyor::reverse).alongWith(Commands.runOnce(intake::push)))
			.onFalse(new StopConveyor().alongWith(new StopIntake()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
	
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}