/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TroTeleOp", group="Magic Shiz")
public class TryTeleOp extends OpMode8696 {

    // Declare OpMode members.
    int direction = -1;
    int addposition = 0;
    double motorpower = 0;
    public double NEW_P = 2.0;
    public static final double NEW_I = 0.0;
    public static final double NEW_D = 0.0;
    public int position = 0;
    PIDCoefficients pidOrig = new PIDCoefficients();
    PIDCoefficients pidModified = new PIDCoefficients();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //initGyro();
        initOtherStuff();
        initDriveTrain();
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        pidOrig = leftPivot.getPIDCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);

        PIDCoefficients pidNew1 = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        PIDCoefficients pidNew2 = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        leftPivot.setPIDCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidNew1);
        rightPivot.setPIDCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidNew2);

        pidModified = leftPivot.getPIDCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //Sets power for the tank drive mecanum wheels. We use the left and right gamepad 1 sticks in order to do this.
        if (gamepad1.b){
            direction = 1;
        }
        if (gamepad1.a){
            direction = -1;
        }

        if (gamepad2.a){
            position = -250;
            motorpower = 1;
        }
        if (gamepad2.b){
            addposition = -150;
        }
        if (gamepad2.left_bumper){
            addposition = 5;
        }
        if (gamepad2.right_bumper){
            addposition = 5;
        }
        while (addposition > 0 || addposition < 0){
            position += addposition;
            if(position < -250 && addposition < 0){
                motorpower = 0.2;
            }
            else if(position < -250 && addposition > 0){
                motorpower = 1;
            }
            else if(position > -250 && addposition > 0){
                motorpower = 0.2;
            }
            else if(position > -250 && addposition < 0){
                motorpower = 1;
            }
            else{
                motorpower = 1;
            }

            addposition = 0;
        }
        rightPivot.setTargetPosition(position);
        leftPivot.setTargetPosition(position);
        rightPivot.setPower(motorpower - 0.02);
        leftPivot.setPower(motorpower);

        leftBack.setPower(-gamepad1.left_stick_y * direction);
        leftFront.setPower(-gamepad1.left_stick_y * direction);
        rightBack.setPower(gamepad1.right_stick_y * direction);
        rightFront.setPower(gamepad1.right_stick_y * direction);

        //Strafing right for the right bumper and left for the left bumper.
        //Helps with aligning our robot to the lander correctly.
        if (gamepad1.left_bumper){
            leftBack.setPower(1 * direction);
            leftFront.setPower(-1 * direction);
            rightBack.setPower(1 * direction);
            rightFront.setPower(-1 * direction);
        }
        if (gamepad1.right_bumper){
            leftBack.setPower(-1 * direction);
            leftFront.setPower(1 * direction);
            rightBack.setPower(-1 * direction);
            rightFront.setPower(1 * direction);
        }

        // Compute speed of left,right motors.

        //All other controls besides movement reside on the second controller. It makes the driver's life easier.

        //Lift controls are set on the Dpad. Originally we had the latch also on the left and right
        //dpad, but the controls would interfere.
        if (gamepad2.dpad_down) lift.setPower(-1);
        if (gamepad2.dpad_up) lift.setPower(1);
        if (!gamepad2.dpad_down && !gamepad2.dpad_up) lift.setPower(0);


        //Slides our linear slide system in and out.

        if (gamepad2.dpad_left){
            dumper.setPower(1);
        }
        if (gamepad2.dpad_right){
            dumper.setPower(-1);
        }
        if (!gamepad2.dpad_right && !gamepad2.dpad_left){
            dumper.setPower(0);
        }

        if(gamepad2.x){
            extender.setPower(1);
        }
        if(gamepad2.y){
            extender.setPower(-1);
        }
        if(!gamepad2.x && !gamepad2.y){
            extender.setPower(0);
        }

        telemetry.addData("Runtime", "%.03f", getRuntime());
        //telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                //pidOrig.p, pidOrig.i, pidOrig.d);
        //telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                //pidModified.p, pidModified.i, pidModified.d);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
