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
import frc.TSLib.leds.LedStrip;
import frc.TSLib.leds.LedStrip.LedStatus;
import frc.TSLib.leds.SetLedState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.compound.AlignToTag;
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
	public static final LedStrip leds = new LedStrip(0, 9);

	// BINDS for buttons
	final Trigger zeroHeadingButton = joystick.leftBumper().and(joystick.rightBumper());
	final Trigger slowButton = joystick.leftTrigger();
	final Trigger alignButton = joystick.a();
	final Trigger shootButton = joystick.rightTrigger();
	final Trigger intakeDownButton = joystick.povDown();
	final Trigger intakeUpButton = joystick.povUp();
	final Trigger reverseButton = joystick.povLeft();
	final Trigger intakeButton = joystick.rightBumper().and(zeroHeadingButton.negate());
	final Trigger confirmButton = joystick.povRight();

	final Trigger inAllianceZone = new Trigger(()->FieldConstants.getScoringZone().contains(getRobotPose().getTranslation()));
	final Trigger inNeutralZone = new Trigger(()->FieldConstants.neutralZone.contains(getRobotPose().getTranslation()));
	final Trigger trenchActivator = new Trigger(
		()->{
			int tid = limelight.getTID();
			int[] tags = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? VisionConstants.trenchTagsBlue : VisionConstants.trenchTagsRed;

			for (int id: tags) {
				if (tid == id) return true;
			}

			return false;
		}
	);
	final Trigger hubActivator = new Trigger(
		()->{
			int tid = limelight.getTID();
			int[] tags = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? VisionConstants.hubTagsBlue : VisionConstants.hubTagsRed;

			for (int id: tags) {
				if (tid == id) return true;
			}
			
			return false;
		}
	);

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

		return angle;
	}

    public Distance getDistanceToHub() {
        Translation2d hub = FieldConstants.getHubPos();
        Translation2d robot = getRobotPose().getTranslation();

        return Meters.of(hub.minus(robot).getNorm());
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
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * slowMult * 2) // Drive counterclockwise with negative X (left)
            )
        );

		alignButton.onFalse(new SetLedState(LedStatus.Off));
		
        alignButton.whileTrue(
			drivetrain.applyRequest(()->face
				.withTargetDirection(getDirectionToHub())
                .withMaxAbsRotationalRate(MaxAngularRate / 1.5)
				.withVelocityX(-joystick.getLeftY() * MaxSpeed * slowMult) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed * slowMult) // Drive left with negative X (left)
			).alongWith(new SetLedState(LedStatus.Ready))
		);

		trenchActivator.debounce(.15)
		.and(alignButton.negate())
			.onTrue(new SetLedState(LedStatus.Target))
			.onFalse(new SetLedState(LedStatus.Off));
		
		// align to trench TODO continue
		//alignbutton && trenchActivator .whileTrue(align to trench -> leds)
		alignButton.and(trenchActivator).whileTrue(
			new AlignToTag().andThen(new SetLedState(LedStatus.Ready))
		);
		
		//alignbutton && allianceZone && !trenchActivator .whileTrue(align to hub -> leds -> confirm -> shoot)
		//alignbutton && neutralZone && !trenchActivator .whileTrue(align to nearest openi	ng to alliance zone -> confirm -> delivery) 

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

        intakeButton.onTrue(new Pull()).onFalse(new StopIntake());

        intakeUpButton.onTrue(new UpdateSetpoint(IntakeConstants.resetPos));
        intakeDownButton.onTrue(new UpdateSetpoint(IntakeConstants.openPos));

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