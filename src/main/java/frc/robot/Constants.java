package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Constants {
	public static class IntakeConstants {
		public static final int angleMotorCANID = 1;
		public static final int intakeMotorCANID = 2;

        public static final int limitSwitchID = 0;

		public static final double intakeSpeed = .6;
		public static final double angleSpeed = .5;

        public static final double gearRatio = 1/100;

		public static final Angle angleForwardSoftLimit = Rotations.of(-1);
		public static final Angle angleReverseSoftLimit = Rotations.of(-113.15);
		
		public static final double angleGearRatio = 1;
		
		public static final Slot0Configs angleS0C = new Slot0Configs()
		.withKP(5).withKI(0).withKD(.2).withKG(0)
		.withGravityType(GravityTypeValue.Arm_Cosine);
		
		public static final Angle resetPos = Rotations.of(0);
		public static final Angle openPos = Rotations.of(-113.12);
	}

    public static class ConveyorConstants {
        public static final int conveyorMotorCANID = 3;
        public static final int alignMotorCANID = 4;

        public static final double conveyorSpeed = .9;
        public static final double alignSpeed = .7;
    }

    public static class ShooterConstants {
        public static final int leaderCANID = 5;
        public static final int followerCANID = 6;

        public static final double ratio = 18/24;

        public static final AngularVelocity idleSpeed = RotationsPerSecond.of(0);

        public static final Slot0Configs S0C = new Slot0Configs()
        .withKP(.25).withKI(0).withKD(0).withKA(0).withKV(0);

        public static final double[][] speedMapData = {
			{1.25, 65},
			{1.7, 70},
        };
    }

    public static class LedConstants {
        public static final int port = 0;
        public static final int count = 9;
    }

    public class VisionConstants {
        public static final String llName = "limelight-front";
		public static final int updateFrequency = 10; // once every x periodic loops
    }

	public class SwerveConstants {
		public static final AngularVelocity maxAngularVelocity = RotationsPerSecond.of(1.5);
		
		public static final double rotateKp = 15;
		public static final double rotateKi = 0;
		public static final double rotateKd = 1;

	}

	public class AutoConstants {
		public static final double driveKp = 0;
		public static final double driveKi = 0;
		public static final double driveKd = 0;
		
		public static final double steerKp = 12;
		public static final double steerKi = 0;
		public static final double steerKd = 0;
	}

	public class FieldConstants {
		public static final Distance zero = Inches.of(0);
		
		public static final Distance fieldWidth = Inches.of(651.22);
		public static final Distance fieldHeight = Inches.of(317.69);
		
		public static final Distance allianceZoneWidth = Inches.of(156.61);
		public static final Distance allianceZoneHeight = fieldHeight;
		
		public static final Distance hubX = Inches.of(182.11);
		public static final Distance hubY = fieldHeight.div(2);
		
		public static final Distance neutralZoneWidth = Inches.of(240);
		public static final Distance neutralZoneHeight = fieldHeight;
		
		public static final Translation2d hubPosBlue = new Translation2d(hubX, hubY);
		public static final Translation2d hubPosRed = new Translation2d(fieldWidth.minus(hubX), hubY);

		
		public static final Rectangle2d field = new Rectangle2d(
			Translation2d.kZero,
			new Translation2d(fieldWidth, fieldHeight)
		);
		
		public static final Rectangle2d scoringZoneBlue = new Rectangle2d(
			Translation2d.kZero,
			new Translation2d(allianceZoneWidth, allianceZoneHeight)
		);

		public static final Rectangle2d scoringZoneRed = new Rectangle2d(
			new Translation2d(fieldWidth, fieldHeight),
			new Translation2d(fieldWidth.minus(allianceZoneWidth), zero)
		);

		public static final Rectangle2d neutralZone = new Rectangle2d(
			new Translation2d(fieldWidth.div(2).minus(neutralZoneWidth.div(2)), fieldHeight),
			new Translation2d(fieldWidth.div(2).plus(neutralZoneWidth.div(2)), zero)
		);

		
		public static final Translation2d getHubPos() {
			return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? hubPosBlue : hubPosRed;
		}

		public static final Rectangle2d getScoringZone() {
			return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? scoringZoneBlue : scoringZoneRed;
		}

	}
}