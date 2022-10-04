package frc.robot.commands;

import static frc.robot.utilities.Util.logf;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class KevinShooterCmd extends CommandBase {

    public enum ShooterMode {
        RUNNING, STOPPED, SHOOT,  SET_SHOOTER_SPEED_SLOW, SET_SHOOTER_SPEED_MEDIUM,
        SET_SHOOTER_SPEED_FAST, SHOOTER_OFF, SET_SHOOTER_VELOCITY, REVERSE_SHOOTER
    };

    private ShooterMode mode = ShooterMode.STOPPED;

    enum State {
        IDLE, START_SHOOT,  WAIT_FOR_SPEED
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
                Robot.compressor.stop();
                break;
            case SET_SHOOTER_SPEED_SLOW:
                Robot.shooter.setShooterSpeed(0.33); // change to .33 only after a test shot
                break;
            case SET_SHOOTER_SPEED_MEDIUM:
                Robot.shooter.setShooterSpeed(0.55);
                break;
            case SET_SHOOTER_SPEED_FAST:
                Robot.shooter.setShooterSpeed(0.65);
                break;
            case SHOOTER_OFF:
                Robot.shooter.setShooterSpeed(0);
                break;
            case SET_SHOOTER_VELOCITY:
                if (data == 0) {
                    Robot.shooter.stopShooter();
                    break;
                }
                Robot.shooter.setShooterVelocity(data);
                break;
            case REVERSE_SHOOTER:
                Robot.shooter.shooterOut();
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
                // See if ball sensors valid and working
                // Make sure shooter speed is valid is not wait a bit for the speed to come up
                if (!isShootSpeedValid()) {
                    state = State.WAIT_FOR_SPEED;
                    delay = 10;
                    return false;
                }
                // Ball is present at rear, front ball moved out of the way, shooter speed OK
                // Everything seems OK so kick fribee into shooter
                delay = 10;
                return false;
            // Wait a bit for the shooter to come up to speed
            case WAIT_FOR_SPEED:
                delay--;
                if (delay < 0) {
                    logf("!!!!! Shooter not up to speed in allocated time\n");
                }
                if (isShootSpeedValid()) {
                    state = State.START_SHOOT;
                }
                return false;
        }
        return true;
    }

    void setShooterSpeed(double speed) {
        Robot.shooter.setShooterSpeed(speed);
    }

    boolean isShootSpeedValid() {
        double tolerance = 800; // The varation of allowed speed
        double speed = Robot.shooter.getShooterSpeed();
        double requestedVelocity = Robot.shooter.getRequestedVelocity();
        if (Math.abs(speed - requestedVelocity) <= tolerance) {
            return true;
        }

        // Shooter req .33 peed:28165 29365 32503 31439
        // Shooter req .55 speed:54144 52960
        // Shooter req .60 speed 57296
        // Shooter req .70 speed 66760 66496
        // Shooter req .80 speed 73984

        // Shooter data
        // .55 Shoots 88.5"
        // .60 shoots 105"

        logf("Shoot speed too low speed:%.2f requested:%.2f\n", speed, requestedVelocity);
        return false;

    }
}
