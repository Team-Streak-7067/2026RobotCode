// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class UpdateSetpoint extends InstantCommand {
	Intake intake = Intake.getInstance();
	Angle setpoint;
	
	public UpdateSetpoint(Angle setpoint) {
		addRequirements(intake);
		this.setpoint = setpoint;
	}
	
	@Override
	public void initialize() {
		intake.updateSetpoint(setpoint);
	}
}