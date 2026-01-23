package frc.TSLib.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Controller interface for XInput devices
 */
public class AnalogController extends Controller {
	final JoystickButton a;
	final JoystickButton b;
	final JoystickButton x;
	final JoystickButton y;
	final JoystickButton rb;
	final JoystickButton lb;
	final JoystickButton rs;
	final JoystickButton ls;
	final JoystickButton start;
	final JoystickButton back;

	public AnalogController(int port) {
		super(port);
		this.a = new JoystickButton(joystick, XboxController.Button.kA.value);
		this.b = new JoystickButton(joystick, XboxController.Button.kB.value);
		this.x = new JoystickButton(joystick, XboxController.Button.kX.value);
		this.y = new JoystickButton(joystick, XboxController.Button.kY.value);
		this.rb = new JoystickButton(joystick, XboxController.Button.kRightBumper.value);
		this.lb = new JoystickButton(joystick, XboxController.Button.kLeftBumper.value);
		this.rs = new JoystickButton(joystick, XboxController.Button.kRightStick.value);
		this.ls = new JoystickButton(joystick, XboxController.Button.kLeftStick.value);
		this.start = new JoystickButton(joystick, XboxController.Button.kStart.value);
		this.back = new JoystickButton(joystick, XboxController.Button.kBack.value);
	}

	public JoystickButton getA() {
		return a;
	}

	public JoystickButton getB() {
		return b;
	}

	public JoystickButton getX() {
		return x;
	}

	public JoystickButton getY() {
		return y;
	}

	public JoystickButton getRB() {
		return rb;
	}

	public JoystickButton getLB() {
		return lb;
	}

	public JoystickButton getRS() {
		return rs;
	}

	public JoystickButton getLS() {
		return ls;
	}

	public JoystickButton getStart() {
		return start;
	}

	public JoystickButton getBack() {
		return back;
	}
	
	public double getLT() {
		return joystick.getRawAxis(XboxController.Axis.kLeftTrigger.value);
	}

	public double getRT() {
		return joystick.getRawAxis(XboxController.Axis.kRightTrigger.value);
	}

	public double getLeftX() {
		return joystick.getRawAxis(XboxController.Axis.kLeftX.value);
	}
	
	public double getLeftY() {
		return joystick.getRawAxis(XboxController.Axis.kLeftY.value);
	}
	
	public double getRightX() {
		return joystick.getRawAxis(XboxController.Axis.kRightX.value);
	}
	
	public double getRightY() {
		return joystick.getRawAxis(XboxController.Axis.kRightY.value);
	}
}