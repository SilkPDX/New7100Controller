package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="BaseAutonomous")
public class BaseAutonomous extends LinearOpMode{
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor SpinnerL;
    DcMotor SpinnerR;
    DcMotor[] motors = new DcMotor[4];

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loadDelay = new ElapsedTime();
    boolean infiniteLoop = false;

    static final double TICKS_PER_CM = 1120 / 31.9186; //cm.
    @Override
    public void runOpMode() throws InterruptedException {
        runSetup();
        waitForStart();

        driveCm(1, 50, 0);
        driveCm(1, 0, 50);
        driveCm(1, 0, -50);
        driveCm(1, -50,0);
        spinSpinners(1,1);

    }

    void runSetup() {
        motorLF = hardwareMap.get(DcMotor.class, "lf_drive");
        motorLB = hardwareMap.get(DcMotor.class, "lb_drive");
        motorRF = hardwareMap.get(DcMotor.class, "rf_drive");
        motorRB = hardwareMap.get(DcMotor.class, "rb_drive");
        motors[0] = motorLF;
        motors[1] = motorLB;
        motors[2] = motorRF;
        motors[3] = motorRB;
        motorRF.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        SpinnerL = hardwareMap.get(DcMotor.class, "spinner_l");
        SpinnerR = hardwareMap.get(DcMotor.class, "spinner_r");
        SpinnerR.setDirection(DcMotor.Direction.REVERSE);
        SpinnerL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SpinnerR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();

    }
    public void moveForward(double power, long seconds){

        motorLB.setPower(power);
        motorLF.setPower(power);
        motorRB.setPower(power);
        motorRF.setPower(power);
        sleep(seconds*1000);
        motorLB.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorRF.setPower(0);
    }

    public void strafe(Boolean isLeft, double power, long seconds){
        if (isLeft){
            power = -power;
        }

        motorLB.setPower(-power);
        motorLF.setPower(power);
        motorRB.setPower(power);
        motorRF.setPower(-power);
        sleep(seconds*1000);
        motorLB.setPower(0);
        motorLF.setPower(0);
        motorRB.setPower(0);
        motorRF.setPower(0);
    }

    public void spinSpinners(double power, long seconds){
        SpinnerL.setPower(power);
        SpinnerR.setPower(power);
        sleep(seconds*1000);
        SpinnerL.setPower(0);
        SpinnerR.setPower(0);
    }


    void driveCm(double speed, int forwardCm, int horizontalCm) {

        int forwardTicks = (int) Math.round(TICKS_PER_CM * forwardCm);
        int horizontalTicks = (int) Math.round(TICKS_PER_CM * horizontalCm);

        int[] targets = {
                motorLF.getCurrentPosition() + forwardTicks + horizontalTicks, //lf
                motorLB.getCurrentPosition() + forwardTicks - horizontalTicks, //lb
                motorRF.getCurrentPosition() + forwardTicks - horizontalTicks, //rf
                motorRB.getCurrentPosition() + forwardTicks + horizontalTicks, //rb
        };



        double[] speeds = new double[4];

        //find the target with greatest magnitude
        int maxTarget = 0;
        for(int i = 0; i < motors.length; i++) {
            if (maxTarget < Math.abs(targets[i])) {
                maxTarget = Math.abs(targets[i]);
            }
        }
        telemetry.addData("max", maxTarget);
        telemetry.update();

        for(int i = 0; i < motors.length; i++) {
            speeds[i] = ((double) targets[i] / maxTarget) * speed; //the fastest wheel should be going at the speed "speed"
        }
        telemetry.addData("speed0", speeds[0]);
        for(int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition(targets[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(speeds[i]);
        }

        while(opModeIsActive() && (motors[0].isBusy() || motors[1].isBusy() || motors[2].isBusy() || motors[3].isBusy())) {
            telemetry.addData("pos0", motors[0].getCurrentPosition());//wait for stuff to done
        }

        for(DcMotor motor: motors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

}
