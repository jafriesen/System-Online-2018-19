package org.firstinspires.ftc.teamcode._Auto.SuperSimpleAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;

@Autonomous(name="Claim Auto", group="SimpleAuto")
public class ClaimAuto extends AutoOpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    Servo marker;

    @Override
    public void setup() {
        motorFrontRight = hardwareMap.dcMotor.get("fr");
        motorFrontLeft = hardwareMap.dcMotor.get("fl");
        motorBackRight = hardwareMap.dcMotor.get("br");
        motorBackLeft = hardwareMap.dcMotor.get("bl");

        marker = hardwareMap.servo.get("marker");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        mSequence.add(new AutoLib.ServoStep(marker, 0.0));
        mSequence.add(new AutoLib.MoveByTimeStep(motorFrontRight, motorBackRight, motorFrontLeft, motorBackLeft, 0.8, 1, true));
        mSequence.add(new AutoLib.ServoStep(marker, 1.0));
        mSequence.add(new AutoLib.MoveByTimeStep(motorFrontRight, motorBackRight, motorFrontLeft, motorBackLeft, -0.8, 0.2, true));
    }
}