package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.teamcode.OpMode8696;

//Created by USER on 2/24/2018.

@Autonomous(name = "FullDepo", group = "test")
public class FullDepo extends OpMode8696 {

    @Override
    public void init(){

        initRobot();
        initDogeCV();

    }
    @Override
    public void init_loop() {

    }
    @Override
    public void start(){

        lift.setPower(1);
        sleep(6620);

        lift.setPower(0);
        sleep(1000);

        leftFront.setPower(-0.3);
        leftBack.setPower(-0.3);
        rightFront.setPower(0.3);
        rightBack.setPower(0.3);

        sleep(700);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(1000);

        rightFront.setPower(-0.7);
        rightBack.setPower(0.7);
        leftFront.setPower(-0.7);
        leftBack.setPower(0.7);

        sleep(1800);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        sleep(500);

        detector.disable();

    }
    @Override
    public void loop(){

    }
    @Override
    public void stop(){

    }

}