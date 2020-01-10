package frc.lib;

import java.util.Timer;
import java.util.TimerTask;

public class TimeOutTimer {
    private Timer timer = new Timer();
    private long timeoutTime = 0;
    private boolean timedOut = false;

    
    public TimeOutTimer(long ms) {
        this.timeoutTime = ms;
    }

    public void start() {
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                timedOut = true;
                stop();
            }
        }, 0, timeoutTime);
    }

    public void stop() {
        timer.cancel();
    }

    public boolean getTimedOut() {
        return timedOut;
    }

    public void setTimeoutTime(long timeoutTime) {
        this.timeoutTime = timeoutTime;
    }

    public void reset() {
        stop();
        timer = new Timer();
        timedOut = false;
    }
 
}