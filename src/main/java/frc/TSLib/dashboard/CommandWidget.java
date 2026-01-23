package frc.TSLib.dashboard;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

public class CommandWidget {
	public static void addWidget(String name, Supplier<Command> command, boolean ignoreDisable) {
		SmartDashboard.putData(
			command.get()
			.withName(name).ignoringDisable(ignoreDisable)
		);
	}
	
	public static void add2StateWidget(String name, Supplier<Command> start, Supplier<Command> end, boolean ignoreDisable) {
		SmartDashboard.putData(
			name,
			new StartEndCommand(
				()->start.get(),
				()->end.get()
			).withName(name).ignoringDisable(ignoreDisable)
		);
	}
}