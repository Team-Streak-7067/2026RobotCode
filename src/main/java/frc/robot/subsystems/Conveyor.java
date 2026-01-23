// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
	static Conveyor instance = new Conveyor();
	
	TalonFX conveyorMotor = new TalonFX(ConveyorConstants.conveyorMotorCANID);
	TalonFX alignMotor = new TalonFX(ConveyorConstants.alignMotorCANID);
	
	public static Conveyor getInstance() {
		return instance;
	}
	
	public Conveyor() {
		configMotors();
	}
	
	@Override
	public void periodic() {
	}

	void configMotors() {
		TalonFXConfiguration convCfg = new TalonFXConfiguration();
		convCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		convCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		conveyorMotor.getConfigurator().apply(convCfg);

		TalonFXConfiguration alignCfg = new TalonFXConfiguration();
		alignCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		alignCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		alignMotor.getConfigurator().apply(alignCfg);
	}

	public void pull() {
		conveyorMotor.set(ConveyorConstants.conveyorSpeed);
	}

	public void stopConveyor() {
		conveyorMotor.stopMotor();
	}

	public void align() {
		alignMotor.set(ConveyorConstants.alignSpeed);
	}

	public void stopAlign() {
		alignMotor.stopMotor();
	}

	public void stopAll() {
		try {
			getCurrentCommand().cancel();
		} catch (Exception e) {}
		stopConveyor();
		stopAlign();
	}
}