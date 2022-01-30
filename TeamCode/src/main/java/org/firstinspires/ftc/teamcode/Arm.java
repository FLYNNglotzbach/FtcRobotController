package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
public class Arm {
    private DcMotor _main_motor;
    private Servo _main_servo;
    private Gamepad _the_gg_gamepad;
    private int _level;
    private static final float SCALING_FACTOR = 1;
    private int previous_count;
    private static final float level_1 = 4;
    private static final float level_2 = 6;
    private static final float level_3 = 12;
    private float slider_position;
    private double servo_initial_position;

    public Arm (DcMotor main_motor,Servo main_servo,Gamepad the_gg_gamepad) {

        _level = 0;
        _main_motor = main_motor;
        _main_servo = main_servo;
        _the_gg_gamepad = the_gg_gamepad;
        previous_count = _main_motor.getCurrentPosition();
        slider_position = 0;
        servo_initial_position = _main_servo.getPosition();


    }
    private void extend(float power) {
        _main_motor.setPower(power);
    }
    private void retract(float power) {
        _main_motor.setPower(-1*power);
    }
    private void stop() {
        _main_motor.setPower(0);
    }
    private void setLevel(int level) {
        _level = level;
    }
    public void service() {

        //check d pad then set level
        if (_the_gg_gamepad.dpad_down) {
            setLevel(0);
        }
        if (_the_gg_gamepad.dpad_left) {
            setLevel(1);
        }
        if (_the_gg_gamepad.dpad_up) {
            setLevel(2);
        }
        if (_the_gg_gamepad.dpad_right) {
            setLevel(3);
        }
        //check rt trigger and extend
        if (_the_gg_gamepad.right_trigger > 0) {
            extend(_the_gg_gamepad.right_trigger);
        } else if (_the_gg_gamepad.left_trigger > 0) {
            retract(_the_gg_gamepad.left_trigger);
        } else {
            stop();
        }
        //check lt trigger and retract

        int current_count = _main_motor.getCurrentPosition();
        slider_position = slider_position + (current_count - previous_count) * SCALING_FACTOR;
        previous_count = current_count;


    }

}



