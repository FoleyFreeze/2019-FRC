package frc.robot.subsystems.vision;

/**
 * Define event sent by PiListener
 * 
 * @param String s
 *            method that is called when an object detect event is received
 *            from a Pi; implemented by the person who defines this interface
 */
public interface PiEvent {
	public void eventGet(String s);
}