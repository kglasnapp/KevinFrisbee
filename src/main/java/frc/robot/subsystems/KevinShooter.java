package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import static frc.robot.utilities.Util.logf;

public class KevinShooter extends SubsystemBase {
  private MotorSRX shooterMotor;
  private double lastSpeed = 0;
  private MotorSRX backspinMotor;
  private PID shooterPID; // PID for shooter velocity
  private double lastShooterVelocity;
  private int lastPOV = -1;

  // Current threshold to trigger current limit
  private int kPeakCurrentAmps = 10;
  // Duration after current exceed Peak Current to trigger current limit
  private int kPeakTimeMs = 0;
  // Current to mantain once current limit has been triggered
  private int kContinCurrentAmps = 5;

  public KevinShooter() {
    shooterMotor = new MotorSRX("Shooter", Robot.config.shooterID, -1, true);

    // shooterPID = new PID("Shooter", 0.045, .00007, .7, 0, 0, -1, 1, false); //
    // Seems to work still a lot of oscillations
    shooterPID = new PID("Shooter", .35, .00002, .5, 0, 0, -1, 1, false); // With overshoot 0.00007 .7
    shooterMotor.setSensorPhase(false);
    shooterMotor.setVelocityPID(shooterPID);
    shooterMotor.setBrakeMode(false);
    shooterMotor.setCurrentLimit(kPeakCurrentAmps, kContinCurrentAmps,kPeakTimeMs);
    logf("Kevin shooter is setup at ID:%d\n", Robot.config.shooterID);
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    int operatorPOV = Joysticks.operator.getPOV();
    // Set shooter speed based upon operator POV
    if (operatorPOV >= 0 && Robot.config.shooterVelocityPID && lastPOV != operatorPOV) {
        double[] shootSpeed = {0, Robot.config.ShooterSpeedPIDLow, Robot.config.ShooterSpeedPIDMedium, Robot.config.ShooterSpeedPIDHigh  };
        double newVelocity = shootSpeed[operatorPOV / 90];
        logf("Set new shooter velocity:%.2f\n", newVelocity);
        setShooterVelocity(newVelocity);
    } 
    if (Robot.count % 500 == 200 && getShooterSpeed() > 0) {
      logf("Shooter speed:%.2f req:%.2\n", getShooterSpeed(), lastSpeed);
    }
    if (Robot.count % 15 == 6) {
      SmartDashboard.putNumber("Sh Sp", getShooterSpeed());
    }
  }

  public void setShooterSpeed(double speed) {
    if (lastSpeed == speed) {
      return;
    }
    logf("New Shooter speed:%.2f last:%.2f\n", speed, lastSpeed);
    shooterMotor.setSpeed(speed);
    lastSpeed = speed;
  }

  public void setShooterVelocity(double velocity) {
    lastShooterVelocity = velocity;
    shooterMotor.setVelocity(velocity);
  }

  public double getShooterSpeed() {
    return shooterMotor.getActualVelocity();
  }

  public double getBackSpinSpeed() {
    return backspinMotor.getActualVelocity();
  }

  double range(double val, double fromMin, double fromMax, double toMin, double toMax) {
    return (val - fromMin) * (toMax - toMin) / (fromMax - fromMin) + toMin;
  }

  public double getRequestedVelocity() {
    return lastShooterVelocity;
  }

  public void stopShooter() {
    shooterMotor.stopMotor();
  }

  public void shooterOut() {
    logf("Start shooter reverse\n");
    shooterMotor.setVelocity(-10020); // arbitrary rpm to spit out ball if stuck
  }

}