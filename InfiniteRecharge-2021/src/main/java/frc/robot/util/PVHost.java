// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;

import edu.wpi.first.wpilibj.DriverStation;

/** 
 * Robot code host for the PathVisualizer application.
 */
public class PVHost {
    private ServerSocket serverSocket;
    private Socket clientSocket;
    private boolean connected;

    /**
     * Creates a new PVHost. It will listen for connections on the specified port.
     * @param port The port that the host will operate on. The port you enter in PathVisualizer should match the one passed here.
     */
    public PVHost(int port) {
        try {
            serverSocket = new ServerSocket(port);
        } catch(IOException ex) {
            DriverStation.reportError("PVHost could not create a ServerSocket!\n" + ex.getMessage(), true);
        }

        attemptToConnectSocket();
    }

    /**
     * Sends the specified robot position to the PathVisualizer client.
     * @param robotPosition The current robot position.
     */
    public void update(Point2D robotPosition) {
        String message = composeMessage("Pos", robotPosition.toString());
        sendMessage(message);
    }

    /**
     * Sends a path to the PathVisualizer client for viewing.
     * @param path The Path to send.
     * @param name The name of the path. Will appear on the manifest with that name.
     */
    public void sendPath(Path path, String name) {
        if(path.isValid()) {
            String message = composeMessage("Path-" + name, path.toString());
            sendMessage(message);
        } else {
            DriverStation.reportError("PVHost could not send a path because it was invalid!", false);
        }
    }

    /**
     * Attempts to connect the host to the client.
     * This method starts a new Thread, so it will return immediately, but the socket may not be connected.
     * Use this.connected to check if the client is connected.
     */
    private void attemptToConnectSocket() {
        connected = false;
        new Thread(
            () -> {
                try {
                    clientSocket = serverSocket.accept();
                    connected = true;
                } catch(IOException ex) {
                    DriverStation.reportError("PVHost could not connect to client!\n" + ex.getMessage(), true);
                }
            }
        ).start();
    }

    /**
     * Sends a message to the client. The message should be formatted using composeMessage().
     * @param message The message to send (should be formatted).
     */
    private void sendMessage(String message) {
        if(connected) {
            try {
                clientSocket.getOutputStream().write(message.getBytes());
            } catch(SocketException ex) {
                //likely due to the client disconnecting. Terminate connection and try to get it going again
                try {
                    clientSocket.close();
                } catch(IOException ex2) {
                    DriverStation.reportError("PVHost could not close client socket!\n" + ex2.getMessage(), true);
                }

                attemptToConnectSocket();
            } catch(IOException ex) {
                DriverStation.reportError("PVHost could not send message!\n" + ex.getMessage(), true);
            }
        }
    }

    /**
     * Formats a message into a format that the client can read.
     * @param subject The subject of the message. Should be "Pos" if sending a position, or "Path-[pathname] if sending a path."
     * @param message The body of the message.
     */
    private String composeMessage(String subject, String message) {
        return "(" + subject + ":" + message + ")";
    }
}
