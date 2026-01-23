package frc.TSLib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * Controller interface for DirectInput devices
 */
public class DigitalController extends Controller {
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
	final JoystickButton lt;
	final JoystickButton rt;

	public DigitalController(int port) {
		super(port);
		x = new JoystickButton(joystick, 1);
		a = new JoystickButton(joystick, 2);
		b = new JoystickButton(joystick, 3);
		y = new JoystickButton(joystick, 4);
		lb = new JoystickButton(joystick, 5);
		rb = new JoystickButton(joystick, 6);
		lt = new JoystickButton(joystick, 7);
		rt = new JoystickButton(joystick, 8);
		back = new JoystickButton(joystick, 9);
		start = new JoystickButton(joystick, 10);
		ls = new JoystickButton(joystick, 11);
		rs = new JoystickButton(joystick, 12);
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

	public JoystickButton getLT() {
		return lt;
	}

	public JoystickButton getRT() {
		return rt;
	}

	public double getLeftX() {
		return joystick.getRawAxis(Joystick.AxisType.kX.value);
	}
	
	public double getLeftY() {
		return joystick.getRawAxis(Joystick.AxisType.kY.value);
	}
	
	public double getRightX() {
		return joystick.getRawAxis(Joystick.AxisType.kZ.value);
	}
	
	public double getRightY() {
		return joystick.getRawAxis(Joystick.AxisType.kTwist.value);
	}
}