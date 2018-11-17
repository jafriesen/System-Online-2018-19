package org.firstinspires.ftc.teamcode._Auto.SuperSimpleAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;

@Autonomous(name="Iffy Latch Auto", group="SimpleAuto")
public class IffyLandAuto extends AutoOpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorLift;
    DcMotor motors[];

    Servo marker;

    @Override
    public void setup() {
        motorFrontRight = hardwareMap.dcMotor.get("fr");
        motorFrontLeft = hardwareMap.dcMotor.get("fl");
        motorBackRight = hardwareMap.dcMotor.get("br");
        motorBackLeft = hardwareMap.dcMotor.get("bl");
        motorLift = hardwareMap.dcMotor.get("lift");
        motors = new DcMotor[1];
        motors[0] = motorLift;
        marker = hardwareMap.servo.get("marker");
        motorLift.setPower(-0.15);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        mSequence.add(new AutoLib.MoveByTimeStep(motors, 1, 1.88, true));
        mSequence.add(new AutoLib.MoveByTimeStep(motorFrontRight, motorBackRight, motorFrontLeft, motorBackLeft, -0.8, 1, true));
    }
}