
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    DcMotor _intake_motor;
    public Intake(DcMotor intake_motor) {
        _intake_motor = intake_motor;

    }
    public void on() {
    _intake_motor.setPower(0.5);
    }

    public void off() {
    _intake_motor.setPower(0);
    }


}