// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax ElevatorBaseMotorOne;
    private CANSparkMax ElevatorBaseMotorTwo;
    private CANSparkMax ExtendingArmMotor;
    private DigitalInput LowerMagneticLimitSwitch;

    /** Creates a new IntakeSubsystem. */
    public ElevatorSubsystem() {
        ElevatorBaseMotorOne = new CANSparkMax(Constants.ELEVATOR_BASE_MOTOR_ONE_CAN_ID, MotorType.kBrushless);
        ElevatorBaseMotorTwo = new CANSparkMax(Constants.ELEVATOR_BASE_MOTOR_TWO_CAN_ID, MotorType.kBrushless);

        LowerMagneticLimitSwitch = new DigitalInput(Constants.LOWER_ELEVATOR_MAGNETIC_LIMIT_SWITCH_CHANNEL);

        ExtendingArmMotor = new CANSparkMax(Constants.EXTENDING_ARM_MOTOR_CAN_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void ElevatorUp() {

    }
}