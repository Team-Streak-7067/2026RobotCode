// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static java.lang.Math.cos;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.TSLib.leds.LedStrip;
import frc.TSLib.leds.LedStrip.LedStatus;
import frc.TSLib.leds.SetLedState;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LedConstants;
import frc.robot.Constants.ShooterConstants;
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
import frc.robot.commands.shooter.SpinUp;
import frc.robot.commands.shooter.StopShooter;
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

	Translation2d shotData = getShotData();

	int[] trenchTags = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? VisionConstants.trenchTagsBlue : VisionConstants.trenchTagsRed;
	int[] hubTags = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? VisionConstants.hubTagsBlue : VisionConstants.hubTagsRed; 

    final CommandXboxController driver = new CommandXboxController(0);
	final CommandXboxController operator = new CommandXboxController(1);

    public static final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    public static final Vision limelight = Vision.getInstance();
    public static final Intake intake = Intake.getInstance();
    public static final Conveyor conveyor = Conveyor.getInstance();
    public static final Shooter shooter = Shooter.getInstance();
	public static final LedStrip leds = new LedStrip(LedConstants.port, LedConstants.count);

	// driver binds
	final Trigger zeroHeadingButton = driver.leftBumper().and(driver.rightBumper());
	final Trigger slowButton = driver.leftTrigger();
	final Trigger alignHubButton = driver.a();
	final Trigger alignTrenchButton = driver.b();
	final Trigger confirmButton = driver.rightTrigger();
	final Trigger reverseButton = driver.povLeft();
	final Trigger intakeMacroButton = driver.rightBumper().and(zeroHeadingButton.negate());
	
	// operator binds
	final Trigger shootButton = operator.rightTrigger();
	final Trigger intakeDownButton = operator.povDown();
	final Trigger intakeUpButton = operator.povUp();
	final Trigger pullButton = operator.rightBumper();
	final Trigger pushButton = operator.leftBumper();

	// field triggers
	final Trigger inAllianceZone = new Trigger(()->FieldConstants.getScoringZone().contains(getRobotPose().getTranslation()));
	final Trigger inNeutralZone = new Trigger(()->FieldConstants.neutralZone.contains(getRobotPose().getTranslation()));
	final Trigger trenchActivator = new Trigger(
		()->{
			int tid = limelight.getTID();
			for (int id: trenchTags) {
				if (tid == id) return limelight.getDistanceToTag().lte(VisionConstants.TrenchAlignDistanceThreshold);
			}
			return false;
		}
	);
	final Trigger hubActivator = new Trigger(
		()->{
			int tid = limelight.getTID();
			for (int id: hubTags) {
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
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			angle = angle.rotateBy(Rotation2d.k180deg);
		}

		return angle;
	}

	Translation2d getShotData() {
		Translation2d target = FieldConstants.getHubPos().minus(getRobotPose().getTranslation());

		ChassisSpeeds robotSpeedsFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, drivetrain.getState().Pose.getRotation());
		Translation2d robotVelocity = new Translation2d(robotSpeedsFieldRelative.vxMetersPerSecond, robotSpeedsFieldRelative.vyMetersPerSecond);

		// TODO maybe uncomment this??
		Distance dist = Meters.of(target.getNorm());
		double idealSpeed = shooter.calcSpeed(dist).in(RPM) * cos(ShooterConstants.shooterAngle.in(Radians)); //ball vx

		target = target.div(dist.in(Meters)).times(idealSpeed);

		// i am lost
		// maybe we dont need cos because we want the final vector norm to be the distance?
		// but somewhere we have to project the ball velocity vector into x & y to not overshoot
		// TODO when this works rewrite this
		// target = target.times(cos(ShooterConstants.shooterAngle.in(Radians)));

		Translation2d shot = target.minus(robotVelocity);
		return shot;
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
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * slowMult)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed * slowMult)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate * slowMult * 2)
            )
        );

		// DRIVER
		alignHubButton.onFalse(new SetLedState(LedStatus.Off).andThen(new StopShooter()));
		confirmButton.onTrue(new Shoot()).onFalse(new StopShooting());
		inAllianceZone.whileTrue(Commands.runOnce(()->{shotData = getShotData();}));
		// inAllianceZone.and(()->shooter.getCurrentCommand() == null).whileTrue(new Idle());
		// inAllianceZone.onFalse(new StopShooter());

		SmartDashboard.putBoolean("trenchActivator", trenchActivator.getAsBoolean());
		
		// trench detection leds
		trenchActivator.debounce(.15)
		.and(alignTrenchButton.or(alignHubButton).negate())
			.onTrue(new SetLedState(LedStatus.Target))
			.onFalse(new SetLedState(LedStatus.Off));
		
		// align to trench
		// alignbutton && trenchActivator .whileTrue(align to trench -> leds)
		alignTrenchButton.and(trenchActivator).and(alignHubButton.negate())
		.whileTrue(
			new AlignToTag().andThen(new SetLedState(LedStatus.Ready))
		);
			
		// align to hub
		//alignbutton && allianceZone && !trenchActivator .whileTrue(align to hub -> leds)
		alignHubButton.and(inAllianceZone).and(alignTrenchButton.negate())
		.whileTrue(new ParallelDeadlineGroup(
				new WaitUntilCommand(hubActivator).andThen(new WaitCommand(VisionConstants.tagDetectToAlignDelay)),
				drivetrain.applyRequest(()->face
					// .withTargetDirection(shotData.getAngle())
					.withTargetDirection(getDirectionToHub())
					.withMaxAbsRotationalRate(MaxAngularRate / 1.5)
					.withVelocityX(-driver.getLeftY() * MaxSpeed * slowMult)
					.withVelocityY(-driver.getLeftX() * MaxSpeed * slowMult)
				),
				new SetLedState(LedStatus.Target),
				// new SpinUp(shotData.getNorm())
				new SpinUp(getDistanceToHub())
			).andThen(new SetLedState(LedStatus.Ready))
		);
		
		//alignbutton && neutralZone && !trenchActivator .whileTrue(align to nearest opening to alliance zone -> confirm -> delivery) 

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading on left bumper press.
        zeroHeadingButton.onTrue(Commands.runOnce(drivetrain::seedFieldCentric));
        slowButton.onChange(Commands.runOnce(()->updateSlow(slowButton.getAsBoolean())));

        intakeMacroButton
		.onTrue(new Pull().andThen(new UpdateSetpoint(IntakeConstants.openPos)))
		.onFalse(new StopIntake());

        reverseButton
		.onTrue(Commands.runOnce(conveyor::reverse).alongWith(Commands.runOnce(intake::push)))
		.onFalse(new StopConveyor().alongWith(new StopIntake()));
		
		// OPERATOR
        shootButton.onTrue(new Shoot()).onFalse(new StopShooting());
        intakeUpButton.onTrue(new UpdateSetpoint(IntakeConstants.resetPos));
        intakeDownButton.onTrue(new UpdateSetpoint(IntakeConstants.openPos));

		pullButton.onTrue(new Pull()).onFalse(new StopIntake());
		pushButton.onTrue(Commands.runOnce(intake::push)).onFalse(new StopIntake());

        drivetrain.registerTelemetry(logger::telemeterize);
    }
	
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}