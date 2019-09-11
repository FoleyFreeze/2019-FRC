package frc.robot.subsystems.vision;
import java.net.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.*;

public class PiListener implements Runnable {
	
	PiEvent event; // Call the listener with this reference
	int port;        // Network port number to which to send data
	String ip_address;
	int timeout;
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
	public PiListener(PiEvent event, int port, String ip_address, int timeout) {
		this.event = event;    // Save event interface provided 
		this.port = port;      // Save port provided
		this.ip_address = ip_address; //Save IP address passed to us
		this.timeout = timeout;
	}

	public PiListener(PiEvent event, int port, String ip_address){
		this(event,port,ip_address,5000);
	}

	public void run() {
		// Be a socket server 
		ServerSocket serverSocket = null;
		String inputLine;
		Socket clientSocket = null;

		BufferedReader in;
		BufferedWriter out;
		try {
			InetAddress addr = InetAddress.getByName(ip_address);
			serverSocket = new ServerSocket(port, BACKLOG, addr); 
			int count = 0;
			while(true) {

				SmartDashboard.putString("SocketStatus", "Waiting for connection.....");

				clientSocket = serverSocket.accept();			
				//clientSocket.setKeepAlive(true);
				clientSocket.setSoTimeout(timeout);

				// Retrieve string from socket 
				in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
				//out = new BufferedWriter(new OutputStreamWriter(clientSocket.getOutputStream()));

				SmartDashboard.putString("SocketStatus","Connection successful");

				//int count = 0;
				
				try{
					while (true) {
						inputLine = in.readLine();    // Get message
						if (inputLine != null){
							SmartDashboard.putString("PiMessage",inputLine);
							event.eventGet(inputLine); // Send message
							//out.write(count++);
							Vision.receiveCountNT.setNumber(++count);
						} else {
							throw new SocketTimeoutException("NullMessage");
						}
					}
				} catch(SocketTimeoutException e){
					//took too long, reset
					System.out.println("Socket timeout at " + Timer.getFPGATimestamp());
					SmartDashboard.putNumber("LastSocketTimeout",Timer.getFPGATimestamp());
					SmartDashboard.putString("SocketStatus", "Timeout");
					SmartDashboard.putString("TimeoutReason",e.getMessage());
					//Vision.recieveCountNT.setNumber(0);

					in.close();
					clientSocket.close();
				}	

			}
		} catch (IOException e) {
			System.out.println("PiListener exception of some sort: exit");
			SmartDashboard.putString("SocketStatus", "Exited due to error: " + e.toString());
			e.printStackTrace();
			//System.exit(1);
		} finally {
			try {
				clientSocket.close();
				serverSocket.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

	}
}