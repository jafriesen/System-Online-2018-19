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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Tank Drive w/ Lift", group="TeleOp")
public class LiftTankDrive extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorLift;


    float power = 0;

    public LiftTankDrive() {

    }

    public void init() {

        motorFrontRight = hardwareMap.dcMotor.get("fr");
        motorFrontLeft = hardwareMap.dcMotor.get("fl");
        motorBackRight = hardwareMap.dcMotor.get("br");
        motorBackLeft = hardwareMap.dcMotor.get("bl");
        motorLift = hardwareMap.dcMotor.get("lift");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
    }


    public void loop() {
        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;
        float rt = gamepad1.right_trigger/2;
        float lt = gamepad1.left_trigger/2;
        float liftPower = 0;

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        left =  (float)scaleInput(left);
        right = (float)scaleInput(right);
        rt = (float)scaleInput(rt);
        lt = (float)scaleInput(lt);



        // clip the right/left values so that the values never exceed +/- 1
        left = Range.clip(left, -1, 1);
        right = Range.clip(right, -1, 1);
        rt = Range.clip(rt,(float)0,(float)0.5);
        lt = Range.clip(lt,(float)0,(float)0.5);
        liftPower = rt - lt;

        // write the values to the motors
        motorFrontRight.setPower(right);
        motorBackRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);
        motorLift.setPower(liftPower);

        /*
         * Send telemetry data back to driver station.
         */
        telemetry.addData("Text", "*** v2.1 ***");
        telemetry.addData("gamepad1:", gamepad1);
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
