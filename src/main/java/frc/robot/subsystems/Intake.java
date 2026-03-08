// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.TSLib.dashboard.CommandWidget;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
	static Intake instance = new Intake();
	
	TalonFX angleMotor = new TalonFX(IntakeConstants.angleMotorCANID);
	TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorCANID);

    DigitalInput limitSwitch = new DigitalInput(IntakeConstants.limitSwitchID);

	PositionVoltage AngleCtrl = new PositionVoltage(0);
	
	Intake() {
		configMotors();
		resetPos();
		CommandWidget.addWidget("Reset intake pos", ()->angleMotor.setPosition(0), true);
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("intake pos", getPosition().in(Rotations));
	}

	public static Intake getInstance() {
		return instance;
	}
	
	void updateDashboard() {
		
	}

	void initDashboard() {
		CommandWidget.addWidget("Reset Intake", ()->resetPos(), true);
		CommandWidget.add2StateWidget(
			"Coast Intake",
			()->angleMotor.setNeutralMode(NeutralModeValue.Coast),
			()->angleMotor.setNeutralMode(NeutralModeValue.Brake),
			true
		);
	}

	void configMotors() {
		TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
		intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		intakeMotor.getConfigurator().apply(intakeConfig);

		TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        //probably dont need reverse soft limit because we reset pos by limit switch every time we close intake
		angleConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
		angleConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.angleForwardSoftLimit.in(Rotations);
		angleConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
		angleConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.angleReverseSoftLimit.in(Rotations);

		angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		angleConfig.Feedback.SensorToMechanismRatio = 1/IntakeConstants.angleGearRatio;

		angleConfig.Slot0 = IntakeConstants.angleS0C;

		angleMotor.getConfigurator().apply(angleConfig);
        
		AngleCtrl.Slot = 0;
	}

	Angle getPosition() {
		return angleMotor.getPosition().asSupplier().get();
	}

	void resetPos() {
		angleMotor.setPosition(IntakeConstants.resetPos);
	}

	public void pull() {
		intakeMotor.set(IntakeConstants.intakeSpeed);
	}

	public void stopIntake() {
		intakeMotor.stopMotor();
	}

	public void angleUp() {
		angleMotor.set(IntakeConstants.angleSpeed);
	}

	public void angleDown() {
		angleMotor.set(-IntakeConstants.angleSpeed);
	}

	public void stopAngle() {
		angleMotor.stopMotor();
	}

	public void updateSetpoint(Angle setpoint) {
		// might need to be changed to this (and remove setcontrol from motorConfig)
		angleMotor.setControl(AngleCtrl.withPosition(setpoint));
	}

	public void stopAll() {
		try {
			getCurrentCommand().cancel();
		} catch (Exception e) {}
		stopAngle();
		stopIntake();
	}
}