package frc.robot.subsystems.vision;

import java.net.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.*;

public class PiListener implements Runnable {
	
	PiEvent event; // Call the listener with this reference
	int port;        // Network port number to which to send data

	/**
	 * Create a PiListener
	 * 
	 * @param event
	 *            method that is called when an object detect event is received
	 *            from a Pi
	 * @param port
	 *            network port that this listener will receive object detect
	 *            events from a Pi
	 */
	public PiListener(PiEvent event, int port) {
		this.event = event;    // Save event interface provided 
		this.port = port;      // Save port provided
	}

	public void run() {
		// Be a socket server 
		ServerSocket serverSocket = null;
		String inputLine;
		Socket clientSocket = null;

		try {
			serverSocket = new ServerSocket(port); // Create socket on specified port

			SmartDashboard.putString("socketWaiting", "Waiting for connection.....");
			
			System.out.println("Waiting for connection.....");

			clientSocket = serverSocket.accept();
			clientSocket.setKeepAlive(true);
			System.out.println("Connection successful");
			System.out.println("Waiting for input.....");

			// Retrieve string from socket 
			BufferedReader in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));

			while (true) {
				inputLine = in.readLine();    // Get message
				if (inputLine != null)
					event.eventGet(inputLine); // Send message
			}
		} catch (IOException e) {
			System.out.println("PiListener exception of some sort: exit");
			System.exit(1);
		} finally {
			try {
				serverSocket.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

	}
}