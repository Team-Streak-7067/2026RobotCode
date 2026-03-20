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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.auto.ShootAuto;
import frc.robot.commands.compound.AlignToTag;
import frc.robot.commands.compound.ReverseFeed;
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
	
    final CommandXboxController driver = new CommandXboxController(0);
	final CommandXboxController operator = new CommandXboxController(1);
	
    public static final CommandSwerveDrivetrain drivetrain = CommandSwerveDrivetrain.getInstance();
    public static final Vision limelight = Vision.getInstance();
    public static final Intake intake = Intake.getInstance();
    public static final Conveyor conveyor = Conveyor.getInstance();
    public static final Shooter shooter = Shooter.getInstance();
	public static final LedStrip leds = new LedStrip(LedConstants.port, LedConstants.count);
	
	public static Translation2d shotVector = shooter.getShotVector(FieldConstants.getHubPos());
	
	// driver binds
	final Trigger zeroHeadingButton = driver.start().and(driver.back());
	final Trigger slowButton = driver.leftTrigger();
	final Trigger alignButton = driver.a();
	final Trigger trenchAlignMacro = driver.b();
	final Trigger confirmButton = driver.rightTrigger();
	final Trigger reverseButton = driver.povLeft();
	final Trigger intakeMacroButton = driver.rightBumper();
	
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
			for (int id: FieldConstants.getTrenchTags()) {
				if (tid == id) return limelight.getDistanceToTag().lte(VisionConstants.TrenchAlignDistanceThreshold);
			}
			return false;
		}
	);
	final Trigger hubActivator = new Trigger(
		()->{
			int tid = limelight.getTID();
			for (int id: FieldConstants.getHubTags()) {
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

    public static Pose2d getRobotPose() {
		return drivetrain.getState().Pose;
	}

	Rotation2d rotateByAlliance(Rotation2d rot) {
		return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? rot.rotateBy(Rotation2d.k180deg) : rot;
	}

	public static Distance getShotNorm() {
		return Meters.of(shotVector.getNorm());
	}

    // Rotation2d getDirectionToHub() {
	// 	return rotateByAlliance(shooter.getShotVector().getAngle());
	// }

    // public Distance getDistanceToHub() {
    //     return Meters.of(shooter.getShotVector().getNorm());
    // }

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
		alignButton.onFalse(new SetLedState(LedStatus.Off).andThen(new StopShooter()));
		confirmButton.onTrue(new Shoot(getShotNorm())).onFalse(new StopShooting());
		inAllianceZone.whileTrue(Commands.runOnce(()->{shotVector = shooter.getShotVector(FieldConstants.getHubPos());}));
		inNeutralZone.whileTrue(Commands.runOnce(()->{
			double distL = FieldConstants.getLowerDeliveryPoint().getDistance(getRobotPose().getTranslation());
			double distU = FieldConstants.getUpperDeliveryPoint().getDistance(getRobotPose().getTranslation());

			Translation2d goalPos = distL <= distU ? FieldConstants.getLowerDeliveryPoint() : FieldConstants.getUpperDeliveryPoint();
			shotVector = shooter.getShotVector(goalPos);
		}));

		SmartDashboard.putBoolean("trenchActivator", trenchActivator.getAsBoolean());
		
		// trench detection leds
		trenchActivator.debounce(.15)
		.and(trenchAlignMacro.or(alignButton).negate())
			.onTrue(new SetLedState(LedStatus.Target))
			.onFalse(new SetLedState(LedStatus.Off));
		
		// align to trench
		// alignbutton && trenchActivator .whileTrue(align to trench -> leds)
		trenchAlignMacro.and(trenchActivator).and(alignButton.negate())
		.whileTrue(
			new AlignToTag().andThen(new SetLedState(LedStatus.Ready))
		);
			
		// align to hub
		// alignbutton && allianceZone && !trenchActivator .whileTrue(align to hub -> leds)
		// this shit will overrun loop for sureee FIXME
		alignButton.and(inAllianceZone).and(trenchAlignMacro.negate())
		.whileTrue(
			new ParallelCommandGroup(
				new SpinUp(getShotNorm()),
				// uncomment if using eeshwark shot vector
				// new SpinUp(RPM.of(shotVector.getNorm())),
				drivetrain.applyRequest(()->face
					// might have to remove rotation
					.withTargetDirection(rotateByAlliance(shotVector.getAngle()))
					.withMaxAbsRotationalRate(MaxAngularRate / 1.5)
					.withVelocityX(-driver.getLeftY() * MaxSpeed * slowMult)
					.withVelocityY(-driver.getLeftX() * MaxSpeed * slowMult)
				),
				new SequentialCommandGroup(
					new SetLedState(LedStatus.Target),
					new WaitUntilCommand(hubActivator).andThen(new WaitCommand(VisionConstants.tagDetectToAlignDelay)),
					new SetLedState(LedStatus.Ready)
				).onlyIf(()->leds.getStatus() != LedStatus.Ready)
			)
		);
		
		//alignbutton && neutralZone && !trenchActivator .whileTrue(align to nearest opening to alliance zone -> confirm -> delivery)
		alignButton.and(inNeutralZone).and(trenchAlignMacro.negate())
		.whileTrue(
			new ParallelCommandGroup(
				new SpinUp(getShotNorm()),
				// uncomment if using eeshwark shot vector
				// new SpinUp(RPM.of(shotVector.getNorm())),
				drivetrain.applyRequest(()->face
					// TODO if doesnt work for both alliances remove rotation
					.withTargetDirection(rotateByAlliance(shotVector.getAngle()))
					.withMaxAbsRotationalRate(MaxAngularRate / 1.5)
					.withVelocityX(-driver.getLeftY() * MaxSpeed * slowMult)
					.withVelocityY(-driver.getLeftX() * MaxSpeed * slowMult)
				),
				new SequentialCommandGroup(
					new SetLedState(LedStatus.Target),
					new WaitUntilCommand(hubActivator).andThen(new WaitCommand(VisionConstants.tagDetectToAlignDelay)),
					new SetLedState(LedStatus.Ready)
				).onlyIf(()->leds.getStatus() != LedStatus.Ready)
			)
		);

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
		.onTrue(new ReverseFeed())
		.onFalse(new StopConveyor().alongWith(new StopIntake()));
		
		// OPERATOR
        shootButton.onTrue(new Shoot(getShotNorm())).onFalse(new StopShooting());
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