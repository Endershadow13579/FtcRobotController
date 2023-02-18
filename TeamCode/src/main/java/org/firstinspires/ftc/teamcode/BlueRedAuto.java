package org.firstinspires.ftc.teamcode;

import android.graphics.RectF;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.tensorflow.lite.support.label.Category;
import org.tensorflow.lite.task.vision.detector.ObjectDetector;
import java.util.List;

@Autonomous(name = "BasicRedBlueOpMode (Blocks to Java)")
<<<<<<< Updated upstream

=======
@Disabled
>>>>>>> Stashed changes
public class BlueRedAuto extends LinearOpMode {


    private BNO055IMU imu;
    private DcMotor lb;
    private DcMotor lf;
    private DcMotor rb;
    private DcMotor rf;
    private DcMotor lift;
    private DistanceSensor sf;
    private Servo lc;
    private Servo rc;
    private DistanceSensor sr;
    private DistanceSensor sl;
    private DistanceSensor sb;

    double COUNTS_PER_INCH;
    ElapsedTime past_point;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        BNO055IMU.Parameters recognition;
        double Counts_Per_Motor_Rev;
        int Drive_Gear_Reduction;
        double Wheel_Circumference_Inches;
        boolean programFinished;
        int position =0;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        lb = hardwareMap.get(DcMotor.class, "lb");
        lf = hardwareMap.get(DcMotor.class, "lf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lift = hardwareMap.get(DcMotor.class, "lift");
        sf = hardwareMap.get(DistanceSensor.class, "sf");
        lc = hardwareMap.get(Servo.class, "lc");
        rc = hardwareMap.get(Servo.class, "rc");
        sr = hardwareMap.get(DistanceSensor.class, "sr");
        sl = hardwareMap.get(DistanceSensor.class, "sl");
        sb = hardwareMap.get(DistanceSensor.class, "sb");

        recognition = new BNO055IMU.Parameters();
        recognition.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        recognition.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(recognition);
        Counts_Per_Motor_Rev = 537.7;
        Drive_Gear_Reduction = 1;
        Wheel_Circumference_Inches = 4.4;
        COUNTS_PER_INCH = (Counts_Per_Motor_Rev * Drive_Gear_Reduction) / Wheel_Circumference_Inches;
        past_point = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        programFinished = false;
        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Orientation: ", imu.getAngularOrientation());
        telemetry.addData("Position Left: ", sl.getDistance(DistanceUnit.MM));
        telemetry.addData("Position Right: ", sr.getDistance(DistanceUnit.MM));

        if((sr.getDistance(DistanceUnit.MM)<400) || (sr.getDistance(DistanceUnit.MM)>sl.getDistance(DistanceUnit.MM)))
        {
            position = 1;

        }
        else if (sr.getDistance(DistanceUnit.MM)<sl.getDistance(DistanceUnit.MM)) {
            position = 2;
        }
        telemetry.addData("Signal Position: ", position);

        telemetry.update();
        // Wait for start command from Driver Station.
        waitForStart();
        if (opModeIsActive()) {

<<<<<<< Updated upstream
=======
<<<<<<< Updated upstream
>>>>>>> Stashed changes
//
//            Drive_to_Point("sb", 9, true);
//
//            Drive_to_Point("sb", 100, true);
//            while (imu.getAngularOrientation().firstAngle > -89) {
//                Drive("tr");
//                telemetry.addData("Orientation: ", imu.getAngularOrientation().firstAngle);
//                telemetry.update();
//            }
//           Drive("n");
<<<<<<< Updated upstream
            //liftEncoder(1, 18, 6, true);
            Drive_to_Point("sb",1000,false);
=======
            liftEncoder(1, 18, 6, true);
            while(sr.getDistance(DistanceUnit.MM)<1500){
                Drive("sl");
                if(sr.getDistance(DistanceUnit.MM)<800){
                    lc.setPosition(0);
                    rc.setPosition(1);

                }
                else{
                    lc.setPosition(1);
                    rc.setPosition(0);

                }
            }
            Drive_to_Point("sr", 780, true);
>>>>>>> Stashed changes
//            Drive_to_Point("sf", 767, true);
//            liftEncoder(1, 10, 6, true);
//            liftEncoder(1, 17, 6, false);
//            Drive_to_Point("sf", 780, false);
=======
            Drive_to_Point("sb",900,false);
            Drive_to_Point("sb",800,false);
            while (imu.getAngularOrientation().firstAngle < -1){
                Drive("tl");
            }
            Drive("n");
            //Stop here if Signal 2:

            if(position ==2){
                // Signal 3
//                Drive_to_Point("sr",500,false);
//                while (imu.getAngularOrientation().firstAngle > 1){
//                    Drive("tr");
//                }
//                Drive("n");
//                Drive_to_Point("sr",200,false);
                // Signal 1
//                Drive_to_Point("sr",1155,false);
//                while (imu.getAngularOrientation().firstAngle > 1){
//                    Drive("tr");
//                }
//                Drive("n");
//                Drive_to_Point("sb",800,false);
//                Drive_to_Point("sr",1155,false);

            }
            else if (position ==1){
                // Signal 1
//                Drive_to_Point("sl",500,false);
//                while (imu.getAngularOrientation().firstAngle > 1){
//                    Drive("tr");
//                }
//                Drive("n");
//                Drive_to_Point("sl",200,false);
                // Signal 1
                Drive_to_Point("sl",1140,false);

                while (imu.getAngularOrientation().firstAngle > -178){
                    Drive("tr");
                }
                Drive("n");
                Drive_to_Point("sr",sr.getDistance(DistanceUnit.MM)-10,false);

            }

>>>>>>> Stashed changes
        }
    }

    /**
     * Describe this function...
     */
    private void liftEncoder(int speed, int heightInches, int timeouts, boolean claw) {
        int newLeftTarget;

        newLeftTarget = (int) (heightInches * COUNTS_PER_INCH);
        lift.setTargetPosition(newLeftTarget);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(speed);
        if (claw) {
            lc.setPosition(0);
            rc.setPosition(1);
        } else {
            lc.setPosition(0.9);
            rc.setPosition(0.1);
        }
        past_point.reset();
        while (past_point.seconds() < timeouts && lift.isBusy()) {
            telemetry.addData("Running To ", newLeftTarget);
            telemetry.addData("Time ", past_point.seconds());
            telemetry.addData("Currently At:", lift.getCurrentPosition());
            telemetry.update();
        }
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void Drive(String x) {
        if (x.equals("f")) {
<<<<<<< Updated upstream
            lb.setPower(0.55);
            lf.setPower(0.56);
            rb.setPower(-0.5);
            rf.setPower(-0.5);
        } else if (x.equals("b")) {
            lb.setPower(-0.33);
            lf.setPower(-0.34);
            rb.setPower(0.28);
            rf.setPower(0.28);
=======
            lb.setPower(.80);
            lf.setPower(.80);
            rb.setPower(-.80);
            rf.setPower(-.8);
        } else if (x.equals("b")) {
            lb.setPower(-.8);
            lf.setPower(-.8);
            rb.setPower(.8);
            rf.setPower(.8);
>>>>>>> Stashed changes
        } else if (x.equals("sr")) {
            lf.setPower(0.75);
            lb.setPower(-0.85);
            rb.setPower(0.8);
            rf.setPower(-0.73);
        } else if (x.equals("sl")) {
            lf.setPower(-0.79);
            lb.setPower(0.72);
            rb.setPower(-0.73);
            rf.setPower(0.75);
        } else if (x.equals("tr")) {
            lb.setPower(0.25);
            lf.setPower(0.25);
            rb.setPower(0.25);
            rf.setPower(0.25);
        } else if (x.equals("tl")) {
            lb.setPower(-0.25);
            lf.setPower(-0.25);
            rb.setPower(-0.25);
            rf.setPower(-0.25);
        } else {
            lb.setPower(0);
            lf.setPower(0);
            rb.setPower(0);
            rf.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void Drive_to_Point(String sense, double point, boolean claw) {
        if (sense.equals("sr")) {
            if (sr.getDistance(DistanceUnit.MM) < point) {
                while (sr.getDistance(DistanceUnit.MM) < point) {
                    if (claw) {
                        lc.setPosition(0);
                        rc.setPosition(1);
                    } else {
                        lc.setPosition(0.9);
                        rc.setPosition(0.1);
                    }
                    Drive("sl");
                    telemetry.addData("Distance: ", sr.getDistance(DistanceUnit.MM));
                    telemetry.update();
                }
            } else {
                while (sr.getDistance(DistanceUnit.MM) > point) {
                    if (claw) {
                        lc.setPosition(0);
                        rc.setPosition(1);
                    } else {
                        lc.setPosition(0.9);
                        rc.setPosition(0.1);
                    }
                    Drive("sr");
                    telemetry.addData("Distance: ", sr.getDistance(DistanceUnit.MM));
                    telemetry.update();
                }
            }
        } else if (sense.equals("sl")) {
            if (sl.getDistance(DistanceUnit.MM) < point) {
                while (sl.getDistance(DistanceUnit.MM) < point) {
                    if (claw) {
                        lc.setPosition(0);
                        rc.setPosition(1);
                    } else {
                        lc.setPosition(0.9);
                        rc.setPosition(0.1);
                    }
                    Drive("sr");
                    telemetry.addData("Distance: ", sl.getDistance(DistanceUnit.MM));
                    telemetry.update();
                }
            } else {
                while (sl.getDistance(DistanceUnit.MM) > point) {
                    if (claw) {
                        lc.setPosition(0);
                        rc.setPosition(1);
                    } else {
                        lc.setPosition(0.9);
                        rc.setPosition(0.1);
                    }
                    Drive("sl");
                    telemetry.addData("Distance: ", sl.getDistance(DistanceUnit.MM));
                    telemetry.update();
                }
            }
        } else if (sense.equals("sb")) {
            if (sb.getDistance(DistanceUnit.MM) < point) {
                while (sb.getDistance(DistanceUnit.MM) < point) {
                    if (claw) {
                        lc.setPosition(0);
                        rc.setPosition(1);
                    } else {
                        lc.setPosition(0.9);
                        rc.setPosition(0.1);
                    }
                    Drive("f");
                    telemetry.addData("Distance: ", sb.getDistance(DistanceUnit.MM));
                    telemetry.update();
                }
            } else {
                while (sb.getDistance(DistanceUnit.MM) > point) {
                    if (claw) {
                        lc.setPosition(0);
                        rc.setPosition(1);
                    } else {
                        lc.setPosition(0.9);
                        rc.setPosition(0.1);
                    }
                    Drive("b");
                    telemetry.addData("Distance: ", sb.getDistance(DistanceUnit.MM));
                    telemetry.update();
                }
            }
        } else if (sense.equals("sf")) {
            if (sf.getDistance(DistanceUnit.MM) < point) {
                while (sf.getDistance(DistanceUnit.MM) < point) {
                    if (claw) {
                        lc.setPosition(0);
                        rc.setPosition(1);
                    } else {
                        lc.setPosition(0.9);
                        rc.setPosition(0.1);
                    }
                    Drive("b");
                    telemetry.addData("Distance: ", sf.getDistance(DistanceUnit.MM));
                    telemetry.update();
                }
            } else {
                while (sf.getDistance(DistanceUnit.MM) > point) {
                    if (claw) {
                        lc.setPosition(0);
                        rc.setPosition(1);
                    } else {
                        lc.setPosition(0.9);
                        rc.setPosition(0.1);
                    }
                    Drive("f");
                    telemetry.addData("Distance: ", sf.getDistance(DistanceUnit.MM));
                    telemetry.update();
                }
            }
        }
        Drive("n");
    }
}
