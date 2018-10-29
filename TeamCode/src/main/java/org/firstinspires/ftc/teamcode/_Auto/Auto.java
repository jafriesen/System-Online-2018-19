package org.firstinspires.ftc.teamcode._Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;

@Autonomous(name="Scrim Auto", group="Autonomous")
public class Auto extends AutoOpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    @Override
    public void setup() {
        motorFrontRight = hardwareMap.dcMotor.get("fr");
        motorFrontLeft = hardwareMap.dcMotor.get("fl");
        motorBackRight = hardwareMap.dcMotor.get("br");
        motorBackLeft = hardwareMap.dcMotor.get("bl");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        mSequence.add(new AutoLib.MoveByTimeStep(motorFrontRight, motorBackRight, motorFrontLeft, motorBackLeft, 0.8, 2.0, true));
        mSequence.add(new AutoLib.MoveByTimeStep(motorFrontRight, motorBackRight, motorFrontLeft, motorBackLeft, -0.8, 1.0, true));
    }
}