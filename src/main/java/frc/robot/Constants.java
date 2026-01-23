package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;

public final class Constants {
	public static class IntakeConstants {
		public static final int angleMotorCANID = 0;
		public static final int intakeMotorCANID = 0;

        public static final int limitSwitchID = 0;

		public static final double intakeSpeed = .5;
		public static final double angleSpeed = .2;

		public static final Angle angleForwardSoftLimit = Rotations.of(0);
		public static final Angle angleReverseSoftLimit = Rotations.of(0);
		
		public static final double angleGearRatio = 1;
		
		public static final Slot0Configs angleS0C = new Slot0Configs()
		.withKP(0).withKI(0).withKD(0).withKG(0)
		.withGravityType(GravityTypeValue.Arm_Cosine);
		
		public static final Angle resetPos = Rotations.of(0);
		public static final Angle openPos = Rotations.of(0);
	}

    public static class ConveyorConstants {
        public static final int conveyorMotorCANID = 0;
        public static final int alignMotorCANID = 0;

        public static final double conveyorSpeed = .3;
        public static final double alignSpeed = .3;
    }
}