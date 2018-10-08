package org.firstinspires.ftc.teamcode._Samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.CustomOpMode;

@Autonomous(name="Example", group="Autonomous")
public class ExampleAuto extends CustomOpMode {

    @Override
    public void init() {
        mHardwareFactory = new AutoLib.RealHardwareFactory(this);

        // create the root Sequence for this autonomous OpMode
        mSequence = new AutoLib.LinearSequence();

        // start out not-done
        bDone = false;
    }
}