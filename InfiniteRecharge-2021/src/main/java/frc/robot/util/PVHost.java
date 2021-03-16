// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.nio.file.Files;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/** 
 * Robot code host for the PathVisualizer application.
 */
public class PVHost {
    private ServerSocket serverSocket;
    private Socket clientSocket;
    private boolean connected;
    private String currentData;

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
        String message = composeMessage(MessageType.POSITION, robotPosition.toString());
        sendMessage(message);
        handleIncomingMessages();
    }

    /**
     * Sends a path to the PathVisualizer client for viewing.
     * @param path The Path to send.
     * @param name The name of the path. Will appear on the manifest with that name.
     */
    public void sendPath(Path path, String name) {
        if(path.isValid()) {
            String message = composeMessage(MessageType.PATH, name, path.toString());
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

    private void redoConnection() {
        //likely due to the client disconnecting. Terminate connection and try to get it going again
        try {
            clientSocket.close();
        } catch(IOException ex2) {
            DriverStation.reportError("PVHost could not close client socket!\n" + ex2.getMessage(), true);
        }

        attemptToConnectSocket();
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
                redoConnection();
            } catch(IOException ex) {
                DriverStation.reportError("PVHost could not send message!\n" + ex.getMessage(), true);
            }
        }
    }

    /**
     * Receives messages from the client and handles them.
     */
    private void handleIncomingMessages() {
        if(!connected) {
            return;
        }

        try {
            //add previously received data to currentData.
            if(clientSocket.getInputStream().available() < 1) {
                return;
            }

            byte[] buffer = new byte[Constants.SOCKET_BUFFER_SIZE];
            clientSocket.getInputStream().read(buffer);
            currentData += new String(buffer);

            //parse data and look for messages.
            //message formatted as such: "[start sequence] [subject] [subject sequence if there is one] [subject contents if applicable] [split sequence] [contents of message] [end sequence]"
            while(currentData.indexOf(Constants.START_SEQUENCE) > -1 && currentData.indexOf(Constants.END_SEQUENCE) > -1) {
                int
                    startSequenceIndex = currentData.indexOf(Constants.START_SEQUENCE),
                    endSequenceIndex = currentData.indexOf(Constants.END_SEQUENCE);
                    
                String relavantData = currentData.substring(startSequenceIndex, endSequenceIndex);
                currentData = currentData.substring(endSequenceIndex + 1);

                String[] completedMessages = relavantData.split(Constants.END_SEQUENCE);
                for(int i=0; i<completedMessages.length; i++) {
                    String completedMessage = completedMessages[i];

                    int splitSequenceIndex = completedMessage.indexOf(Constants.SPLIT_SEQUENCE);
                    if(splitSequenceIndex > -1) { //now it is defintely a full message
                        String subject = completedMessage.substring(0, splitSequenceIndex);
                        String message = completedMessage.substring(splitSequenceIndex + Constants.SPLIT_SEQUENCE.length());
                        String extraInfo = "";

                        if(subject.startsWith(Constants.START_SEQUENCE)) {
                            subject = subject.substring(Constants.START_SEQUENCE.length()); //substring off "("
                        }

                        if(subject.contains(Constants.SUBJECT_SEQUENCE)) {
                            extraInfo = subject.substring(subject.indexOf(Constants.SUBJECT_SEQUENCE) + Constants.SUBJECT_SEQUENCE.length());
                        }

                        handleMessage(subject, message, extraInfo);
                    }
                }
            }
        } catch(SocketException ex) {
            redoConnection(); //SocketExceptions are usually caused by the host disconnecting or some other comms problem.
        } catch(SocketTimeoutException ex) { //to stop if from printing a stack trace every time the read times out
        } catch(IOException ex) {
            DriverStation.reportError("PVHost could not read the client data!", true);
        }
    }

    /**
     * Handles a singular message.
     * @param subject The subject of the message in string form.
     * @param message The body of the message.
     */
    private void handleMessage(String subject, String message, String extraInfo) {
        MessageType messageType = MessageType.fromString(subject);
        switch(messageType) {
            case DIRECTORY_REQUEST: { //return a message with all contents of the directory separated by newlines
                    String[] paths = Util.getFilesInDirectory(message, true);
                    String returnMessage = "";
                    for(String path : paths) {
                        returnMessage += path + "\n";
                    }

                    sendMessage(composeMessage(MessageType.DIRECTORY_REQUEST, returnMessage));
                }
                break;
            case LOAD: {
                    if(!Files.exists(java.nio.file.Path.of(message))) {
                        sendMessage(composeMessage(MessageType.LOAD, "ERR"));
                        break;
                    }

                    try {
                        sendMessage(composeMessage(MessageType.LOAD, Files.readString(java.nio.file.Path.of(message))));
                    } catch(IOException ex) {
                        sendMessage(composeMessage(MessageType.LOAD, "ERR"));
                    }
                }
                break;
            case SAVE: {
                    String fileContents = message;
                    java.nio.file.Path filePath = java.nio.file.Path.of(extraInfo);

                    try {
                        Files.writeString(filePath, fileContents);
                        sendMessage(composeMessage(MessageType.SAVE, extraInfo, "OK"));
                    } catch(IOException ex) {
                        sendMessage(composeMessage(MessageType.SAVE, extraInfo, "ERR"));
                    }
                }
                break;
            default:
                DriverStation.reportError("PVHost could not handle message of type \"" + messageType.getCode() + "\"!", false);
                return;
        }
    }

    /**
     * Formats a message into a format that the client can read.
     * @param subject The subject of the message. Should be "Pos" if sending a position, or "Path-[pathname] if sending a path."
     * @param message The body of the message.
     */
    private String composeMessage(MessageType subject, String message) {
        return Constants.START_SEQUENCE + subject.getCode() + Constants.SPLIT_SEQUENCE + message + Constants.END_SEQUENCE;
    }

    /**
     * Formats a message into a format that the client can read.
     * @param subject The subject of the message. Should be "Pos" if sending a position, or "Path-[pathname] if sending a path."
     * @param subjectInfo additional information needed for the client to carry out the task depicted by the message.
     * @param message The body of the message.
     */
    private String composeMessage(MessageType subject, String subjectInfo, String message) {
        return Constants.START_SEQUENCE + subject.getCode() + Constants.SUBJECT_SEQUENCE + subjectInfo + Constants.SPLIT_SEQUENCE + message + Constants.END_SEQUENCE;
    }
}
