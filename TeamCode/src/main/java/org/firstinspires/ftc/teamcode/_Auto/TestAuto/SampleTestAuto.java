package org.firstinspires.ftc.teamcode._Auto.TestAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;
import org.firstinspires.ftc.teamcode._Auto.Steps.SampleStep;
import org.firstinspires.ftc.teamcode._Libs.VuforiaLib_RoverRuckus;

@Autonomous(name="Sample Auto", group="Test")
public class SampleTestAuto extends AutoOpMode {
    @Override
    public void setup() {
        mSequence.add(new SampleStep(mVlib, this));
    }
}