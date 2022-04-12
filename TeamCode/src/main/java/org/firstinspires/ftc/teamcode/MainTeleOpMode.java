package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MainTeleOpMode")
public class MainTeleOpMode extends LinearOpMode{
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    /*DcMotor Spinner;
    Servo IntakeLoad;
    DcMotor intakeB;
    DcMotor intakeF;*/

    @Override public void runOpMode() {
        initializeProgram();
        runProgram();
    }


    private void initializeProgram(){
        initializeMotors();
        waitForStart();
    }

    private void runProgram() {
        while (opModeIsActive()) {

            //drive strafe rotation
            double drive = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotation = -gamepad1.right_stick_x;

            double[] speeds = {
                    (drive + strafe + rotation), //FL
                    (drive - strafe - rotation), //FR
                    (drive - strafe + rotation), //BL
                    (drive + strafe - rotation), //BR
            };

            double max = 0;
            for(double speed:speeds) {
                if (max < Math.abs(speed) ) {
                    max = Math.abs(speed);
                }
            }

            /*if (gamepad1.a) {
                Spinner.setPower(1);
            } else {
                Spinner.setPower(0);
            }

            if (gamepad1.b) {
                Spinner.setPower(-1);
            } else {
                Spinner.setPower(0);
            }

            //Load intake
            if (gamepad1.x){
                intakeB.setPower(1);
            } else {
                intakeB.setPower(0);
            }

            //Set intake with servo
            if (gamepad1.y) {
                IntakeLoad.setPosition(0.3);
            } else {
                IntakeLoad.setPosition(0.6);
            }
            telemetry.addData("position", IntakeLoad.getPosition());

            //Release
            if (gamepad1.right_trigger > 0.5){
               intakeF.setPower(.05);
            } else {
                intakeF.setPower(0);
            }
            */
            if (max > 1) {
                for(int i = 0; i < speeds.length; i++) {
                    speeds[i] /= max;
                }
            }



            //this is the bottom of the code, don't write new code below this point
            motorLF.setPower(speeds[0]);
            motorRF.setPower(speeds[1]);
            motorLB.setPower(speeds[2]);
            motorRB.setPower(speeds[3]);
            telemetry.update();
        }
    }

    private DcMotor[] getMotors(int frontBack, int rightLeft)
    {
        DcMotor[][][] returnArray = new DcMotor[][][] {
                new DcMotor[][] {
                        new DcMotor[] {
                                motorLB
                        },
                        new DcMotor[] {
                                motorRB, motorLB
                        },
                        new DcMotor[] {
                                motorRB
                        }
                },
                new DcMotor[][] {
                        new DcMotor[] {
                                motorLF, motorLB
                        },
                        new DcMotor[] {
                                motorRF, motorLF, motorRB, motorLB
                        },
                        new DcMotor[] {
                                motorRF, motorRB
                        }
                },
                new DcMotor[][] {
                        new DcMotor[] {
                                motorLF
                        },
                        new DcMotor[] {
                                motorRF, motorLF
                        },
                        new DcMotor[] {
                                motorRF
                        }
                }
        };
        return returnArray[frontBack + 1][rightLeft + 1];
    }

    private void initializeMotors() {
        //Drive Train
        motorLF = hardwareMap.get(DcMotor.class, "lf_drive");
        motorLB = hardwareMap.get(DcMotor.class, "lb_drive");
        motorRF = hardwareMap.get(DcMotor.class, "rf_drive");
        motorRB = hardwareMap.get(DcMotor.class, "rb_drive");
        motorRF.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setPower(0);
        motorLB.setPower(0);
        motorRB.setPower(0);
        motorRF.setPower(0);

        //intake setup
        /*IntakeLoad = hardwareMap.get(Servo.class, "IntakeLoad");
        IntakeLoad.setPosition(0.6);
        intakeB = hardwareMap.get(DcMotor.class, "intake_b");
        intakeF = hardwareMap.get(DcMotor.class, "intake_f");
        intakeB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeB.setDirection(DcMotor.Direction.REVERSE);

        //Spinner
        Spinner = hardwareMap.get(DcMotor.class, "spinner_l");
        Spinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
    }
}