package frc.TSLib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.POVButton;

abstract class Controller {
	final Joystick joystick;
	// POVs start at angle 0 pointing up and go clockwise
	final POVButton POV0;		// TOP
	final POVButton POV45;		// TOP RIGHT
	final POVButton POV90;		// RIGHT
	final POVButton POV135;		// BOTTOM RIGHT
	final POVButton POV180;		// BOTTOM
	final POVButton POV225;		// BOTTOM LEFT
	final POVButton POV270;		// LEFT
	final POVButton POV315;		// TOP LEFT
	
	public Controller(int port) {
		this.joystick = new Joystick(port);
		POV0 = new POVButton(joystick, 0);
		POV45 = new POVButton(joystick, 45);
		POV90 = new POVButton(joystick, 90);
		POV135 = new POVButton(joystick, 135);
		POV180 = new POVButton(joystick, 180);
		POV225 = new POVButton(joystick, 225);
		POV270 = new POVButton(joystick, 270);
		POV315 = new POVButton(joystick, 315);
	}

	/**
	 * @return {@code}POVButton{@code} for d-pad up
	 */
	public POVButton getPOV0() {
		return POV0;
	}

	/**
	 * @return {@code}POVButton{@code} for d-pad up-right
	 */
	public POVButton getPOV45() {
		return POV45;
	}

	/**
	 * @return {@code}POVButton{@code} for d-pad right
	 */
	public POVButton getPOV90() {
		return POV90;
	}

	/**
	 * @return {@code}POVButton{@code} for d-pad down-right
	 */
	public POVButton getPOV135() {
		return POV135;
	}

	/**
	 * @return {@code}POVButton{@code} for d-pad down
	 */
	public POVButton getPOV180() {
		return POV180;
	}

	/**
	 * @return {@code}POVButton{@code} for d-pad down-left
	 */
	public POVButton getPOV225() {
		return POV225;
	}

	/**
	 * @return {@code}POVButton{@code} for d-pad left
	 */
	public POVButton getPOV270() {
		return POV270;
	}

	/**
	 * @return {@code}POVButton{@code} for d-pad up-left
	 */
	public POVButton getPOV315() {
		return POV315;
	}
}