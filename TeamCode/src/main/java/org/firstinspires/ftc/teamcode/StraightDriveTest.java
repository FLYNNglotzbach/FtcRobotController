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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.lang.Math;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

/* Demonstrates empty OpMode
 */
@TeleOp(name = "Concept: Granger gurus", group = "Concept")
//@Disabled
public class ConceptNullOp extends OpMode {
    DcMotor right_front;
    DcMotor left_front;
    DcMotor right_back;
    DcMotor left_back;
    Servo duck_servo;
    static final int RED_ALLIANCE = 1;
    static final int BLUE_ALLIANCE = 0;
    int alliance = BLUE_ALLIANCE;

    private ElapsedTime runtime = new ElapsedTime();

    public ConceptNullOp() {

    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        right_front = hardwareMap.dcMotor.get("right_front");
        left_front = hardwareMap.dcMotor.get("left_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_back = hardwareMap.dcMotor.get("left_back");
        duck_servo = hardwareMap.servo.get("duck");
    }

    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {
        right_front.setDirection(DcMotor.Direction.REVERSE);
        left_front.setDirection(DcMotor.Direction.FORWARD);
        right_back.setDirection(DcMotor.Direction.REVERSE);
        left_back.setDirection(DcMotor.Direction.FORWARD);
    }

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {
        double angle = Math.atan2(gamepad1.left_stick_y,-1*gamepad1.left_stick_x);
        double drive_magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        double drive_power1 = drive_magnitude * Math.sin(angle-(1.0/4.0 * Math.PI));
        double drive_power2 = drive_magnitude * Math.sin(angle+(1.0/4.0 * Math.PI));
        double turn_power = gamepad1.right_stick_x;


        if ((gamepad1.left_stick_y == 0) && (gamepad1.left_stick_x == 0)) {
            drive_power1 = 0;
            drive_power2 = 0;
        }

        if ((gamepad1.left_stick_y == 0) && (gamepad1.left_stick_x == 0) &&
            gamepad1.right_stick_x == 0) {
            left_front.setPower(0);
            right_front.setPower(0);
            right_back.setPower(0);
            left_back.setPower(0);
        } else {
            left_front.setPower(drive_power2 - turn_power);
            right_front.setPower(drive_power1 + turn_power);
            right_back.setPower(drive_power2 + turn_power);
            left_back.setPower(drive_power1 - turn_power);
        }



        if (gamepad1.y) {
            if (this.alliance == BLUE_ALLIANCE) {
                duck_servo.setPosition(1);
            } else {
                duck_servo.setPosition(0);
            }
        } else {
            duck_servo.setPosition(0.5);
        }

        if (gamepad1.right_bumper) {
            if (this.alliance == BLUE_ALLIANCE) {
                this.alliance = RED_ALLIANCE;
            } else {
                this.alliance = BLUE_ALLIANCE;
            }
        }

        telemetry.addData("Alliance", this.alliance==BLUE_ALLIANCE ? "Blue":"Red");
    }
}
// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.

package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Drive Encoder", group="Exercises")
//@Disabled
public class DriveWithEncoder extends LinearOpMode
{
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        // You will need to set this based on your robot's
        // gearing to get forward control input to result in
        // forward motion.
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        // reset encoder counts kept by motors.
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors to run forward for 5000 encoder counts.
        leftMotor.setTargetPosition(5000);
        rightMotor.setTargetPosition(5000);

        // set motors to run to target encoder position and stop with brakes on.
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 25% power. Movement will start. Sign of power is
        // ignored as sign of target encoder position controls direction when
        // running to position.

        leftMotor.setPower(0.25);
        rightMotor.setPower(0.25);

        // wait while opmode is active and left motor is busy running to position.

        while (opModeIsActive() && leftMotor.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-fwd-left", leftMotor.getCurrentPosition() + "  busy=" + leftMotor.isBusy());
            telemetry.addData("encoder-fwd-right", rightMotor.getCurrentPosition() + "  busy=" + rightMotor.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        // wait 5 sec to you can observe the final encoder position.

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-left-end", leftMotor.getCurrentPosition());
            telemetry.addData("encoder-fwd-right-end", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // From current position back up to starting point. In this example instead of
        // having the motor monitor the encoder we will monitor the encoder ourselves.

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setTargetPosition(0);
        rightMotor.setTargetPosition(0);

        // Power sign matters again as we are running without encoder.
        leftMotor.setPower(-0.25);
        rightMotor.setPower(-0.25);

        while (opModeIsActive() && leftMotor.getCurrentPosition() > leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder-back-left", leftMotor.getCurrentPosition());
            telemetry.addData("encoder-back-right", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }

        // set motor power to zero to stop motors.

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        resetStartTime();

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-back-left-end", leftMotor.getCurrentPosition());
            telemetry.addData("encoder-back-right-end", rightMotor.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}