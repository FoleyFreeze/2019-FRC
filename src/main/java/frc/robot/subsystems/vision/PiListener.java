package frc.robot.subsystems.vision;
import java.net.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.*;

public class PiListener implements Runnable {
	
	PiEvent event; // Call the listener with this reference
	int port;        // Network port number to which to send data
	String ip_address;
	private static final int BACKLOG = 10;

	
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
	public PiListener(PiEvent event, int port, String ip_address) {
		this.event = event;    // Save event interface provided 
		this.port = port;      // Save port provided
		this.ip_address = ip_address; //Save IP address passed to us
	}

	public void run() {
		// Be a socket server 
		ServerSocket serverSocket = null;
		String inputLine;
		Socket clientSocket = null;

        BufferedReader in;
		try {
            InetAddress addr = InetAddress.getByName(ip_address);

            serverSocket = new ServerSocket(port, BACKLOG, addr); 
            //serverSocket = new ServerSocket();
            serverSocket.setReuseAddress(true);

			SmartDashboard.putString("SocketStatus", "Waiting for connection.....");

            clientSocket = serverSocket.accept();
            clientSocket.setKeepAlive(true);
            SmartDashboard.putString("SocketStatus","Connection successful");

			// Retrieve string from socket 
			in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));

			while (true) {
                inputLine = in.readLine();    // Get message
                if (inputLine != null){
                    SmartDashboard.putBoolean("BufferEmpty",!in.ready());
                    event.eventGet(inputLine); // Send message
                }
            }
		} catch (IOException e) {
            System.out.println("PiListener exception of some sort: exit");
            e.printStackTrace();
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