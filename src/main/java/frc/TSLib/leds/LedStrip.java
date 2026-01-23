package frc.TSLib.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStrip extends SubsystemBase {
	final int length;
	final int port;
	final AddressableLED led;
	final AddressableLEDBuffer buffer;
	
	byte counter = 0;
	LedStatus status;
	
	public LedStrip(int port, int length) {
		this.port = port;
		this.length = length;
		this.led = new AddressableLED(port);
		this.buffer = new AddressableLEDBuffer(length);
		led.setLength(length);
		led.setData(buffer);
		setStatus(LedStatus.Idle);
	}

	@Override
	public void periodic() {
		switch (status) {
			case Idle:
				if (counter == 0) {
					setGradient(Color.kRed, Color.kOrange);
					counter++;
				}
				break;
			case Waiting:
				if (counter == 0) {
					setColor(Color.kAquamarine);
					counter++;
				}
				break;
		}
	}

	public void setColor(Color color) {
		for (int i = 0; i < length; i++) {
			buffer.setLED(i, color);
		}
		led.setData(buffer);
	}

	public void setGradient(Color color1, Color color2) {
		for (int i = 0; i < length; i++) {
			buffer.setLED(i, Color.lerpRGB(color1, color2, (i+.5)/length));
		}
		led.setData(buffer);
	}

	public void on() {
		led.start();
	}

	public void off() {
		led.stop();
	}

	public void setStatus(LedStatus status) {
		this.status = status;
		on();
		counter = 0;
	}

	public LedStatus getStatus() {
		return status;
	}

	public enum LedStatus {
		Idle,
		Waiting
	}
}