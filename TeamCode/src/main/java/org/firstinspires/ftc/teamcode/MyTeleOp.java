package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by pdkainth on 10/22/2017.
 */

@TeleOp(name="Manual Control")
public class MyTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;


    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");

        leftDrive = hardwareMap.get(DcMotor.class, "Motor1");
        rightDrive = hardwareMap.get(DcMotor.class, "Motor2");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void init_loop (){

    }

    @Override
    public void start () {
    runtime.reset();
    }

    @Override
    public void loop () {
        double leftPower;
        double rightPower;

        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;

        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("gamepad1", "drive(%.2f) turn(%.2f", drive, turn);




    }

    @Override
    public void stop (){

    }

}
