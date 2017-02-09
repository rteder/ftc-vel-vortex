package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

class CachedEncoderMonitor extends Thread {
    private DcMotor target;
    private int lastReadPosition = 0;
    private long lastLoopTime = 0;

    private String TAG;

    CachedEncoderMonitor(DcMotor target) {
        this.target = target;
    }

    void loop() throws InterruptedException {
        lastReadPosition = target.getCurrentPosition();

        Thread.sleep(10);
    }

    public void run() {
        try {
            while(true) {
                loop();
            }
        } catch (InterruptedException e) {
            Log.i("CEM", "Shutting down");
        }
    }

    int getPosition() {
        return lastReadPosition;
    }

    long getLastLoopTime() {
        return lastLoopTime;
    }
}
