package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.utilities.Util.logf;

public class PhotonVision extends SubsystemBase implements VisionData {

    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("gloworm");
    double angle = 0;
    double lastAngle = 0;
    boolean noCamera = true;

    public double tx;
    public double ty;
    public double ta;
    public boolean tv;
    public boolean lightState = false;

    public PhotonVision() {
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = null;
        try {
            result = camera.getLatestResult();
        } catch (Exception e) {
            if (Robot.count % 500 == 0)
                logf("!!!!!!!!!!!  No Vision Camera\n");
            return;
        }
        if (Robot.count % 50 == 23) {
            camera.setLED(VisionLEDMode.kOn);
            lightState = true;
        }
        tv = result.hasTargets();
        if (Robot.count % 20 == 10) {
            SmartDashboard.putBoolean("tv", tv);
        }
        if (tv) {
            PhotonTrackedTarget best = result.getBestTarget();
            tx = best.getYaw();
            ty = best.getPitch();
            ta = best.getArea();
            if (Robot.count % 20 == 0) {
                SmartDashboard.putNumber("tx", tx);
                SmartDashboard.putNumber("ty", ty);
                SmartDashboard.putNumber("ta", ta);
            }
        }

    }

    void changePipeline(int id) {
        // Change pipeline to new pipe line
        camera.setPipelineIndex(id);
    }

    public double getX() {
        return tx;
    }

    public double getY() {
        return ty;
    }

    public boolean getV() {
        return tv;
    }

    public double getArea() {
        return ta;
    }

    // Update State of vision light
    public void cameraLight(boolean state) {
        state = lightState;
        if (state)
            camera.setLED(VisionLEDMode.kOn);
        else
            camera.setLED(VisionLEDMode.kOff);
    }
    
    public void takeSnapshot() {
    };
}
