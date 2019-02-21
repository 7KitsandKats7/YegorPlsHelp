/* package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.OpMode8696;

/**
 * Created by USER on 2/24/2018.


@Autonomous(name = "Depo", group = "test")
public class Depo extends OpMode8696 {

    @Override
    public void runOpMode(){

        initRobot();
        initDogeCV();

        waitForStart();

        lift.setPower(1);
        sleep(1420);

        lift.setPower(0);
        sleep(1000);

        leftFront.setPower(0.3);
        leftBack.setPower(0.3);
        rightFront.setPower(-0.3);
        rightBack.setPower(-0.3);

        sleep(700);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(1000);

        rightFront.setPower(0.7);
        rightBack.setPower(-0.7);
        leftFront.setPower(0.7);
        leftBack.setPower(-0.7);

        sleep(2200);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(500);

        leftFront.setPower(1);
        leftBack.setPower(1);
        rightFront.setPower(-1);
        rightBack.setPower(-1);

        sleep(2000);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(1000);

        dumper.setPower(1);
        sleep(2000);

        dumper.setPower(0);
        sleep(1000);

        detector.disable();

    }

}
*/