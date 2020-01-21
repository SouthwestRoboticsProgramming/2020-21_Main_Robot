package frc.lib.Looper;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.Looper.Loop;

public class Looper {

    private Loop loop;
    private long loopTime = 100;
    private boolean looping;

    public Looper(Loop loop, long loopTime) {
        this.loop = loop;
        this.loopTime = loopTime;
        looping = false;
    }
    
    /**
     * Returns if loop is curently looping. 
     *
     * @return boolean asto state of looping. 
     */
    public boolean getLooping() {
        return looping;
    }

    /**
     * Returns the time of the loop.
     *
     * @return The loopTime 
     */
    public double getLoopTime() {
        return loopTime;
    }

    /**
     * Start looping
     */
    public void start() {
        if (!looping) {
            loop.onStart();
            looping = true;
            looper(loop);
            
        }
    }

    /**
     * Stop looping.
     */
    public void stop() {
        if (looping) {
            looping = false;
            loop.onStop();
        }
    }

    private void looper(Loop loop) {
        new Thread(new Runnable() {
            public void run() {
                while (looping && DriverStation.getInstance().isEnabled()){
                    loop.onLoop();
                    try {
                        Thread.sleep(loopTime);
                    } catch (Exception e) {}
                }
            }
        }).start();
    }
}
