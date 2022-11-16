package frc.robot.utilities;

import java.io.*;
import java.math.BigInteger;
import java.net.*;
import java.io.IOException;

import static frc.robot.utilities.Util.logf;

// TODO work with Ellie to see if get this to work -- receive is blocking and probablly needs to run in a thread

public class UDPReceiver extends Thread {
    public static String lastDataReceived = "";
    protected DatagramSocket socket = null;
    protected BufferedReader in = null;
    protected boolean moreQuotes = true;
    InetAddress addr = null;
    int messages = 0;
    byte[] receiveData;
    byte[] sendData;

    public void run() {
        while (true) {
            logf("Start Thread\n");
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e1) {
                e1.printStackTrace();
            }
            logf("End Thread\n");
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e1) {
                e1.printStackTrace();
            }
        }
    }

    void getData() {
        try {
            socket = new DatagramSocket(45009); // Choose a port for your BeagleBone to send packets to!
        } catch (SocketException e) {
            logf("Unable to start UDP thread  %s\n", e.toString());
            e.printStackTrace();
        }
        receiveData = new byte[256];
        sendData = new byte[256];
        DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
        try {
            socket.receive(receivePacket);
        } catch (IOException e) {
            e.printStackTrace();
        }
        String incoming = new String(receivePacket.getData());
        System.out.println("RECEIVED: " + incoming);
    }

    public String toHex(String arg) {
        return String.format("%040x", new BigInteger(1, arg.getBytes()));
    }

    public void sendUDP(String message) {
        int port = 500;
        int length = 10;
        byte[] buff = new byte[10];
        DatagramPacket packet = new DatagramPacket(buff, length, addr, port);
        try {
            socket.send(packet);
        } catch (IOException e) {
            e.printStackTrace();
            return;
        }
    }

}