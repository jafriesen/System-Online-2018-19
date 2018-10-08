package org.firstinspires.ftc.teamcode._Libs;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;

public class CustomOpMode extends OpMode {
    public TelemetryPacket packet = new TelemetryPacket();
    public AutoLib.Sequence mSequence;             // the root of the sequence tree
    public boolean bDone;                          // true when the programmed sequence is done
    public FtcDashboard mDashboard = FtcDashboard.getInstance();
    public AutoLib.HardwareFactory mHardwareFactory;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            packet.put("sequence finished", "");

        mDashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        super.stop();
    }
}