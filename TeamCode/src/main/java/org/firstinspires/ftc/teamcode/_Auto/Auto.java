package org.firstinspires.ftc.teamcode._Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotDrive;
import org.firstinspires.ftc.teamcode._Libs.AutoLib;
import org.firstinspires.ftc.teamcode._Libs.AutoOpMode;
import org.firstinspires.ftc.teamcode._Libs.RoadRunnerImplementer;

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