package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public class Biobuzz2026 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Motores nomeados pela posição no robô.
        // As strings entre aspas precisam bater com a configuração do Driver Hub.
        DcMotor frontLeft  = hardwareMap.dcMotor.get("Mecanum 1"); // porta 0 - frente esquerda
        DcMotor frontRight = hardwareMap.dcMotor.get("Mecanum 2"); // porta 1 - frente direita
        DcMotor backLeft   = hardwareMap.dcMotor.get("Mecanum 3"); // porta 2 - trás esquerda
        DcMotor backRight  = hardwareMap.dcMotor.get("Mecanum 4"); // porta 3 - trás direita

        // Inverte o lado direito.
        // Se o robô andar para trás quando mandar ir para frente,
        // inverta o lado esquerdo em vez do direito.
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU do Control Hub
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Ajuste conforme a montagem do hub no robô
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // eixo Y do analógico vem invertido
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Botão options (start no controle Xbox) zera a direção "frente do campo"
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotaciona o comando do joystick contra o ângulo do robô (field-centric)
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // compensa o strafe imperfeito das mecanum

            // Mantém todas as potências entre -1 e 1 sem perder a proporção
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower  = (rotY + rotX + rx) / denominator;
            double backLeftPower   = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            // Telemetria para conferir no Driver Hub
            telemetry.addData("Heading (graus)", Math.toDegrees(botHeading));
            telemetry.addData("FL / FR", "%.2f / %.2f", frontLeftPower, frontRightPower);
            telemetry.addData("BL / BR", "%.2f / %.2f", backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}