package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public interface MotorDef {
   // public MotorSrx(String name, int Id, int followId, boolean logging);

    public int getPos();

    public void enableLimitSwitch(boolean forward, boolean reverse);

    public boolean getForwardLimitSwitch();

    public boolean getReverseLimitSwitch();

    void setBrakeMode(boolean value);

    void setPos(double position);

    void setVelocity(double velocity);

    public void setInverted(boolean invert);

    public double getLastSpeed();

    public double getActualVelocity();

    //public TalonSRX getMotorSRX();

    void periodic();

    public void logPeriodic();

    /*
     * Peak Current and Duration must be exceeded before current limit is activated.
     * When activated, current will be limited to Continuous Current. Set Peak
     * Current params to 0 if desired behavior is to immediately current-limit.
     */
    public void setCurrentLimit(int peakAmps, int continousAmps, int durationMilliseconds);

    public void updateSmart();

    public void setSpeed(double speed);

    public void setSpeedAbsolute(double speed);

    public void stopMotor();

    void zeroEncoder();

    void setEncoderPosition(double position);

    void setPositionPID(PID pid, FeedbackDevice feedBack);

    void setVelocityPID(PID pid);

    double getMotorVoltage();

    public void PIDToMotor( PID pid, int slot, int timeout);

    public double getError();

    public void logMotorVCS();

    public String getMotorsVCS();

    public double getMotorCurrent();

    public void setSensorPhase(boolean phase);

    // Config the sensor used for Primary PID and sensor direction
    public void setPositionPID(int pidIdx, PID pid);

    void setRampCloseLoopRamp(double rate);

    void setRampOpenLoopRamp(double rate);
}
