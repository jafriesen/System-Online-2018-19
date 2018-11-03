/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import java.util.List;

import static com.sun.tools.doclint.Entity.and;

@TeleOp(name="Tank Drive", group="TeleOp")
public class BasicTankDrive extends OpMode {

    DcMotorEx motorFrontRight;
    DcMotorEx motorFrontLeft;
    DcMotorEx motorBackRight;
    DcMotorEx motorBackLeft;

	public void init() {
		motorFrontLeft = hardwareMap.get(DcMotorEx.class, "fl");
		motorBackLeft = hardwareMap.get(DcMotorEx.class, "bl");
		motorBackRight = hardwareMap.get(DcMotorEx.class, "br");
		motorFrontRight = hardwareMap.get(DcMotorEx.class, "fr");

		motorFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
		motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);

		motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		motorBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		motorBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		motorFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		motorFrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		motorFrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
	}


	public void loop() {
		float left = gamepad1.left_stick_y;
		float right = gamepad1.right_stick_y;

		// scale the joystick value to make it easier to control
		// the robot more precisely at slower speeds.
		//left =  (float)scaleInput(left);
		//right = (float)scaleInput(right);

		// clip the right/left values so that the values never exceed +/- 1
		left = Range.clip(left, -1, 1);
		right = Range.clip(right, -1, 1);

		boolean slowMode = gamepad1.left_bumper;

		if(slowMode) {
            motorFrontRight.setPower(right/2);
            motorBackRight.setPower(right/2);
            motorFrontLeft.setPower(left/2);
            motorBackLeft.setPower(left/2);
        }
        else {
            motorFrontRight.setPower(right);
            motorBackRight.setPower(right);
            motorFrontLeft.setPower(left);
            motorBackLeft.setPower(left);
        }


		/*
		 * Send telemetry data back to driver station.
		 */
		telemetry.addData("Text", "*** v2.1 ***");
		telemetry.addData("gamepad1:", gamepad1);
		telemetry.addData("front left:", motorFrontLeft.getCurrentPosition());
		telemetry.addData("back left:", motorBackLeft.getCurrentPosition());
		telemetry.addData("front right:", motorFrontRight.getCurrentPosition());
		telemetry.addData("back right:", motorBackRight.getCurrentPosition());
	}

	public void stop() {

	}

	/*
	 * This method scales the joystick input so for low joystick values, the 
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
	double scaleInput(double dVal)  {
		return dVal*dVal*dVal;		// maps {-1,1} -> {-1,1}
	}
}
