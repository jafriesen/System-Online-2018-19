package org.firstinspires.ftc.teamcode._Samples;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.FeedforwardTuningOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode._Robot.RobotDrive;

@Autonomous
public class SampleFeedforwardTuningOpMode extends FeedforwardTuningOpMode {
    public SampleFeedforwardTuningOpMode() {
        // TODO: change the following to match your drive
        super(100.0, RobotDrive.MOTOR_CONFIG.getMaxRPM(), 4.0);
    }

    @Override
    protected Drive initDrive() {
        return new RobotDrive(hardwareMap);
    }
}
