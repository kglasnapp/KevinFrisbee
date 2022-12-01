
package frc.robot.utilities;

import java.io.*;
import java.math.BigInteger;
import java.net.*;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

//import com.google.gson.Gson;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UDPReceiver implements Runnable {
    public static String lastDataReceived = "";
    public static double angle;
    public static double distance;
    public static String targetId;
    public static double targetX;

    protected DatagramSocket socket = null;
    protected BufferedReader in = null;
    protected boolean moreQuotes = true;

    public UDPReceiver() throws IOException {
        this("udpReceiver");
    }

    public UDPReceiver(String name) throws IOException {
        socket = new DatagramSocket(9005);
    }

    public void run() {
        JSONParser parser = new JSONParser();
        Util.log("Start UDP thread");
        byte[] buf = new byte[256];
        DatagramPacket packet = new DatagramPacket(buf, buf.length);
        int count = 0;
        Util.logf("Going to start the UDP Server:%s\n", 0);
        while (moreQuotes) {
            try {
                // receive request
                packet.setLength(buf.length);
                socket.receive(packet);
                byte[] data = packet.getData();
                lastDataReceived = new String(data, 0, packet.getLength());
                SmartDashboard.putString("last Data Received", lastDataReceived);
                // Util.logf("Data from UPD:%s\n", lastDataReceived);
                JSONObject jsonObject = (JSONObject) parser.parse(lastDataReceived);
                distance = (double) jsonObject.get("distance");
                angle = (double) jsonObject.get("angle");
                //targetX = (double) jsonObject.get("x");
                targetId = (String) jsonObject.get("target");
                if (count % 25 == 16) {
                    SmartDashboard.putNumber("TagAngle", Util.round2(angle));
                    SmartDashboard.putNumber("TagDistance", Util.round2(distance));
                    SmartDashboard.putString("TagId",targetId);
                    count = 0;
                   // Util.logf("Data from UPD:%s\n", lastDataReceived);
                }
                count++;
            } catch (Exception e) { 
                e.printStackTrace();
                moreQuotes = false;
            }
        }
        socket.close();
    }

    public String toHex(String arg) {
        return String.format("%040x", new BigInteger(1, arg.getBytes()));
    }

}