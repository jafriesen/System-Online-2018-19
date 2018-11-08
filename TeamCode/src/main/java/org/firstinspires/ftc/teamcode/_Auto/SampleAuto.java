package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;
import org.firstinspires.ftc.teamcode._Auto.SampleStep;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;

@Autonomous(name="Sample Auto", group="Autonomous")
public class SampleAuto extends AutoOpMode {

    VuforiaLib_RoverRuckus mVlib;

    @Override
    public void setup() {
        mVlib = new VuforiaLib_RoverRuckus();
        mVlib.init(this, null);

        mSequence.add(new SampleStep(mVlib, this));
    }

    @Override
    public void start(){
        mVlib.start();
    }

    @Override
    public void stop(){
        mVlib.stop();
    }
}