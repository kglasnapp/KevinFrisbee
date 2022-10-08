package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class KevinShooterCmd extends CommandBase {

    public enum ShooterMode {
        RUNNING, STOPPED, SHOOT, SET_SHOOTER_SPEED_SLOW, SET_SHOOTER_SPEED_MEDIUM,
        SET_SHOOTER_SPEED_FAST, SHOOTER_OFF, SET_SHOOTER_SPEED, REVERSE_SHOOTER
    };

    private ShooterMode mode = ShooterMode.STOPPED;

    enum State {
        IDLE, START_SHOOT, DROP_FRISBEE, PUSH_FRISBEE, WAIT_FOR_SPEED, SHOOTER_REVERSE, RUMBLE
    };

    private State state = State.IDLE;
    private int delay = 0; // used to do timing for error time out
    private int overallDelay = 500;

    private double data; // Input data received type of data depends on the mode

    public KevinShooterCmd(ShooterMode mode) {
        // Use requires() here to declare subsystem dependencies
        // addRequirements(Robot.intake);
        this.mode = mode;
    }

    // For mode == SET_SHOOTER_SPEED data is the speed for the shooter -1 to + 1
    public KevinShooterCmd(ShooterMode mode, double data) {
        // Use requires() here to declare subsystem dependencies
        // addRequirements(Robot.intake);
        this.mode = mode;
        this.data = data;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Robot.shooter == null) {
            logf("!!!!! Shooter not setup mode:%s\n", mode);
            return; // If no shooter return
        }
        logf("Shooter mode:%s\n", mode);
        overallDelay = 500;
        switch (mode) {
            case RUNNING:
                break;
            case STOPPED:
                break;
            case SHOOT:
                state = State.START_SHOOT;
                // If after 5 seconds the shoot sequence is not complete something is very wrong
                overallDelay = 50 * 5;
                if (Robot.config.enableCompressor) {
                    Robot.compressor.stop();
                }
                break;
            case SET_SHOOTER_SPEED_SLOW:
                Robot.shooter.setShooterSpeed(Robot.config.ShooterSpeedLow); // change to .33 only after a test shot
                break;
            case SET_SHOOTER_SPEED_MEDIUM:
                Robot.shooter.setShooterSpeed(Robot.config.ShooterSpeedMedium);
                break;
            case SET_SHOOTER_SPEED_FAST:
                Robot.shooter.setShooterSpeed(Robot.config.ShooterSpeedHigh);
                break;
            case SHOOTER_OFF:
                Robot.shooter.setShooterSpeed(0);
                break;
            case SET_SHOOTER_SPEED:
                if (data == 0) {
                    Robot.shooter.stopShooter();
                    break;
                }
                Robot.shooter.setShooterSpeed(data);
                break;
            case REVERSE_SHOOTER:
                Robot.shooter.shooterReverse();
                delay = 50 * 2;
                break;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logf("Shoot sequence ended state:%s overAllDelay:%d\n", state, overallDelay);
        state = State.IDLE;
        if (Robot.compressor != null)
            Robot.compressor.restart();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        switch (state) {
            case IDLE:
                return true;
            case START_SHOOT:
                state = State.DROP_FRISBEE;
                Robot.shooter.activateDroper();
                logf("Start Shoot sequence\n");
                delay = 10;
                return false;
            case DROP_FRISBEE:
                delay--;
                if (delay < 0) {
                    Robot.shooter.releaseDroper();
                    logf("Frisbee dropped\n");
                    state = State.WAIT_FOR_SPEED;
                }
                return false;

            case WAIT_FOR_SPEED:
                delay--;
                if (isShootSpeedValid()) {
                    Robot.shooter.activatePusher();
                    state = State.PUSH_FRISBEE;
                    logf("Push Frisbee into shooter\n");
                    delay = 10;
                }
                if (delay < 0) {
                    logf("!!!!! Shooter not up to speed in allocated time\n");
                    state = State.RUMBLE;
                    delay = 25;
                    Robot.joysticks.setLeftRumble(1);
                    Robot.joysticks.setRightRumble(1);
                    return false;
                }
                return false;

            case PUSH_FRISBEE:
                delay--;
                if (delay < 0) {
                    // Frisbee should have been launched
                    Robot.shooter.releasePusher();
                    state = State.IDLE;
                    return true;
                }
                return false;

            case SHOOTER_REVERSE:
                delay--;
                if (delay < 0) {
                    Robot.shooter.stopShooter();
                    state = State.IDLE;
                    return true;
                }
                return false;

            case RUMBLE:
                delay--;
                if (delay < 0) {
                    Robot.joysticks.setLeftRumble(0);
                    Robot.joysticks.setRightRumble(0);
                    logf("Rumble off\n");
                    return true;
                }
                return false;
        }
        return true;
    }

    void setShooterSpeed(double speed) {
        Robot.shooter.setShooterSpeed(speed);
    }

    boolean isShootSpeedValid() {
        double current = Robot.shooter.getMotorCurrent();
        if (current < 5.0 && current > 2.0) {
            return true;
        }
        logf("!!!!! Shooter current too high or low for shooting current:%.2f\n", current);
        return false;
    }
}
