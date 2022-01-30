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
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Arm;

/* Demonstrates empty OpMode
 */
@TeleOp(name = "Competition", group = "Official")
//@Disabled
public class GurusMainOpMode extends OpMode {
    DcMotor right_front;
    DcMotor left_front;
    DcMotor right_back;
    DcMotor left_back;
    Servo duck_servo;
    static final int RED_ALLIANCE = 1;
    static final int BLUE_ALLIANCE = 0;
    int alliance = BLUE_ALLIANCE;
    DcMotor intake_motor;
    Intake intake;
    DcMotor sliding_motor;
    Arm arm;
    DcMotor main_motor;
    Servo lift_servo;
    double _previous_time;
    double _previous_right_front_position;
    double _previous_left_front_position;
    double _previous_right_back_position;
    double _previous_left_back_position;
    private static final double MAX_SPEED = 2.9;
    private boolean _previous_gamepad_right_bumper;
    private ElapsedTime runtime = new ElapsedTime();

    public GurusMainOpMode() {

    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Setup Drive Chassis
        right_front = hardwareMap.dcMotor.get("right_front");
        left_front = hardwareMap.dcMotor.get("left_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_back = hardwareMap.dcMotor.get("left_back");

        // Setup Duck Servo
        duck_servo = hardwareMap.servo.get("duck");

        // Setup Intake
        intake_motor = hardwareMap.dcMotor.get("intake_motor");
        intake = new Intake(intake_motor);

        // Setup Arm
        sliding_motor = hardwareMap.dcMotor.get("sliding_motor");
        lift_servo = hardwareMap.servo.get("lift_servo");
        //arm = new Arm(sliding_motor, lift_servo, gamepad1);
        _previous_gamepad_right_bumper = false;
    }


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
        double time = System.currentTimeMillis();
        double current_right_front_position = right_front.getCurrentPosition();
        double current_left_front_position = left_front.getCurrentPosition();
        double current_right_back_position = right_back.getCurrentPosition();
        double current_left_back_position = left_back.getCurrentPosition();
        double right_front_speed = (current_right_front_position - _previous_right_front_position) / (time - _previous_time);
        double left_front_speed = (current_left_front_position - _previous_left_front_position) / (time - _previous_time);
        double right_back_speed = (current_right_back_position - _previous_right_back_position) / (time - _previous_time);
        double left_back_speed = (current_left_back_position - _previous_left_back_position) / (time - _previous_time);
        _previous_time = time;
        _previous_right_front_position = current_right_front_position;
        _previous_left_front_position = current_left_front_position;
        _previous_right_back_position = current_right_back_position;
        _previous_left_back_position = current_left_back_position;

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

        //arm.service();

        // handle duck harvesting
        if (gamepad1.y) {
            if (this.alliance == BLUE_ALLIANCE) {
                duck_servo.setPosition(1);
            } else {
                duck_servo.setPosition(0);
            }
        } else {
            duck_servo.setPosition(0.5);
        }
        if (gamepad1.a) {
            intake.on();
        }
        else {
            intake.off();
        }

        // new right_bumper button handling - duck turntable direction
        if ((gamepad1.right_bumper) && (_previous_gamepad_right_bumper != gamepad1.right_bumper)) {
            if (this.alliance == BLUE_ALLIANCE) {
                this.alliance = RED_ALLIANCE;
            } else {
                this.alliance = BLUE_ALLIANCE;
            }
        }
        _previous_gamepad_right_bumper = gamepad1.right_bumper;

        telemetry.addData("Alliance", this.alliance==BLUE_ALLIANCE ? "Blue":"Red");
        telemetry.addData("rf speed",right_front_speed);
        telemetry.addData("lf speed",left_front_speed);
        telemetry.addData("rb speed",right_back_speed);
        telemetry.addData("lb speed",left_back_speed);
    }
}
