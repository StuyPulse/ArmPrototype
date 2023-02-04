package com.stuypulse.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.ArmFeedforward;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.streams.filters.MotionProfile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VoltageArm extends SubsystemBase {

    private CANSparkMax motor;

    private RelativeEncoder relativeEncoder;
    // private AbsoluteEncoder absoluteEncoder;

    private SmartNumber setpoint;
    private SmartNumber kG;
    private SmartNumber kP;

    private SmartNumber kMaxVelocity;
    private SmartNumber kMaxAcceleration;

    private Controller controller;

    private SmartBoolean zeroing;

    public VoltageArm() {
        motor = new CANSparkMax(1, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(true);

        motor.enableVoltageCompensation(10);

        relativeEncoder = motor.getEncoder();
        
        relativeEncoder.setPositionConversionFactor(360 * (1 / 36.0));
        relativeEncoder.setPosition(-90);

        setpoint = new SmartNumber("Setpoint", 0);
        kP = new SmartNumber("kP", 0.3);
        kG = new SmartNumber("kG", 0.25);

        kMaxVelocity = new SmartNumber("Max Velocity (deg per s)", 10);
        kMaxAcceleration = new SmartNumber("Max Acceleration (deg per s^2)", 20);

        controller = new PIDController(kP, 0, 0)
            .add(new ArmFeedforward(kG))
            .setSetpointFilter(new MotionProfile(kMaxVelocity, kMaxAcceleration));

        zeroing = new SmartBoolean("Zero?", false);
    }

    public double getDegrees() {
        return relativeEncoder.getPosition();
    }

    @Override
    public void periodic() {
        if (zeroing.get()) relativeEncoder.setPosition(0);

        controller.update(setpoint.get(), relativeEncoder.getPosition());
        motor.setVoltage(controller.getOutput());
    
        SmartDashboard.putNumber("Output", controller.getOutput());
        SmartDashboard.putNumber("Angle (deg)", relativeEncoder.getPosition());
    }
    
}
