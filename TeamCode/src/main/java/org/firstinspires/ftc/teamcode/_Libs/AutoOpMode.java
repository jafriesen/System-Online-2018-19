package org.firstinspires.ftc.teamcode._Libs;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;

public class AutoOpMode extends OpMode {
    public TelemetryPacket packet = new TelemetryPacket();
    public AutoLib.Sequence mSequence;             // the root of the sequence tree
    public boolean bDone;                          // true when the programmed sequence is done
    public FtcDashboard mDashboard = FtcDashboard.getInstance();
    public AutoLib.HardwareFactory mHardwareFactory;
    public VuforiaLib_RoverRuckus mVlib;

    @Override
    public void init() {
        mHardwareFactory = new AutoLib.RealHardwareFactory(this);

        mDashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        mVlib = new VuforiaLib_RoverRuckus();
        mVlib.init(this, null);

        setup();

        // start out not-done
        bDone = false;
    }

    @Override
    public void loop() {
        // until we're done, keep looping through the current Step(s)
        if (!bDone)
            bDone = mSequence.loop();       // returns true when we're done
        else
            packet.put("sequence finished", "");

        packet.put("packet", 1);
        mDashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        mVlib.stop();
        super.stop();
    }

    @Override
    public void start(){
        mVlib.start();
    }

    public void setup() {}
}