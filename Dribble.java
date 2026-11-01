//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//@TeleOp(name="Dribble")
//
//public class Dribble extends LinearOpMode {
//    private DcMotor leftFrontMotor;
//    private DcMotor leftBackMotor;
//    private DcMotor r/* Copyright (c) 2017 FIRST. All rights reserved.
//     *
//     * Redistribution and use in source and binary forms, with or without modification,
//     * are permitted (subject to the limitations in the disclaimer below) provided that
//     * the following conditions are met:
//     *
//     * Redistributions of source code must retain the above copyright notice, this list
//     * of conditions and the following disclaimer.
//     *
//     * Redistributions in binary form must reproduce the above copyright notice, this
//     * list of conditions and the following disclaimer in the documentation and/or
//     * other materials provided with the distribution.
//     *
//     * Neither the name of FIRST nor the names of its contributors may be used to endorse or
//     * promote products derived from this software without specific prior written permission.
//     *
//     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//     * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//     * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//     */
//
// package org.firstinspires.ftc.teamcode;
//
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
// import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.IMU;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.util.ElapsedTime;
// import com.qualcomm.robotcore.util.Range;
//    /*
//     * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
//     * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
//     * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
//     * class is instantiated on the Robot Controller and executed.
//     *
//     * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
//     * It includes all the skeletal structure that all linear OpModes contain.
//     *
//     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
//     * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
//     */
//
//    @TeleOp(name="Dribble")
//
//    public class Dribble extends LinearOpMode {
//
//        private DcMotor frontLeft;
//        private DcMotor frontRight;
//        private DcMotor backLeft;
//        private DcMotor backRight;
//        private double kSpeed=1;
//        private Gamepad gamepad;
//        @Override
//        public void runOpMode() throws InterruptedException {
//            // 初始化电机
//            gamepad = gamepad1;
//            frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
//            frontRight = hardwareMap.get(DcMotor.class, "rightFront");
//            backLeft = hardwareMap.get(DcMotor.class, "leftBack");
//            backRight = hardwareMap.get(DcMotor.class, "rightBack");
//
//            // 设置电机方向
//            frontLeft.setDirection(DcMotor.Direction.REVERSE);
//            backLeft.setDirection(DcMotor.Direction.REVERSE);
//            frontRight.setDirection(DcMotor.Direction.FORWARD);
//            backRight.setDirection(DcMotor.Direction.FORWARD);
//            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//
//
//            waitForStart();
//        private void Chassis(){
//
//
//            if(gamepad1.right_bumper){
//                kSpeed=0.4;
//            } else{
//                kSpeed=1;
//            }
//            double y = -gamepad1.left_stick_y;
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x*0.8;
//
//
//
//
//            if (gamepad1.dpad_up) {
//                rotX = 0;
//                rotY = 0.9;
//            }
//            else if (gamepad1.dpad_down) {
//                rotX = 0;
//                rotY = -0.9;
//            }
//            else if (gamepad1.dpad_left) {
//                rotX = -0.9;
//                rotY = 0;
//            }
//            else if (gamepad1.dpad_right) {
//                rotX = 0.9;
//                rotY = 0;
//            }
//
//
//
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = kSpeed*(rotY + rotX + rx) / denominator;
//            double backLeftPower = kSpeed*(rotY - rotX + rx) / denominator;
//            double frontRightPower = kSpeed*(rotY - rotX - rx) / denominator;
//            double backRightPower = kSpeed*(rotY + rotX - rx) / denominator;
//
//
//            frontLeft.setPower(frontLeftPower);
//            backLeft.setPower(backLeftPower);
//            frontRight.setPower(frontRightPower);
//            backRight.setPower(backRightPower);
//
//            telemetry.addData("Front Left Power", frontLeftPower);
//            telemetry.addData("Back Left Power", backLeftPower);
//            telemetry.addData("Front Right Power", frontRightPower);
//            telemetry.addData("Back Right Power", backRightPower);
//            telemetry.addData("Robot Heading", Math.toDegrees(botHeading));
//            telemetry.addData("重置方向", "按A键重置车头方向");
//
//        }
//    }
//    ightFrontMotor;
//    private DcMotor rightBackMotor;
//    private Gamepad gamepad;
//    @Override
//
//    public void runOpMode(){
//        gamepad = gamepad1;
//        leftFrontMotor = hardwareMap.get(DcMotor.class,"leftFrontMotor");
//        leftBackMotor = hardwareMap.get(DcMotor.class,"leftBackMotor");
//        rightFrontMotor = hardwareMap.get(DcMotor.class,"rightFrontMotor");
//        rightBackMotor = hardwareMap.get(DcMotor.class,"rightBackMotor");
//
//        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        waitForStart();
//        while (opModeIsActive()){
//
//            double y = -gamepad1.left_stick_y;
//            double x = gamepad1.left_stick_x;
//            double rx = gamepad1.right_stick_x;
//
//            double leftFrontpower = (y - x + rx);
//            double leftBackpower = (y + x + rx);
//            double rightFrontpower = (y + x - rx);
//            double rightBackpower = (y - x - rx);
//
//            leftFrontMotor.setPower(leftFrontpower);
//            leftBackMotor.setPower(leftBackpower);
//            rightFrontMotor.setPower(rightFrontpower);
//            rightBackMotor.setPower(rightBackpower);
//
//
//
//        }
//    }
//}
///* y正/负  前进/后退
//               x正/负  右移/左移1
//               rx正/负 顺时针转/逆时针转 */
//
///* y让四个轮子同向*/
//
//            /* x>0 左前、右后 = 正方向
//                   左后、右前 = 负方向 */
//            /* x<0 左前、右后 = 负方向
//                   左后、右前 = 正方向 */
//
//            /* rx>0（右转）
//               左侧 = 正方向
//               右侧 = 负方向*/
//            /* rx<0（左转）
//               左侧 = 负方向
//               右侧 = 正方向*/
//
//            /* 程序大意为，先定义摇杆，保证正负方向与正负值相同。
//            每0.02秒获取一次摇杆的数值，根据获取到的x y rx值来执行。 + 或者 - 要根据电机怎么转来看。
//            例如，当左摇杆向左推时，x获取到的值是正，此时左前电机应该正转，因此是+x；
//            如果此时左前电机应该反转，就应该是-x */
//
