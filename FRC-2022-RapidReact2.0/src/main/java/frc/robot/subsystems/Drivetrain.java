// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotMap;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class Drivetrain extends SubsystemBase {
    private TalonFX leftMaster;
    private TalonFX leftSlave;
    private TalonFX rightMaster;
    private TalonFX rightSlave;

    public DifferentialDrive drive;

    public Drivetrain(){
        leftMaster = new TalonFX(RobotMap.kLeftMaster); //CAN 0
        leftSlave = new TalonFX(RobotMap.kLeftSlave); //CAN 1
        rightMaster = new TalonFX(RobotMap.kRightMaster); //CAN 2
        rightSlave = new TalonFX(RobotMap.kRightSlave); //CAN 3

        leftMaster.setInverted(false);
        leftSlave.setInverted(false);
        rightMaster.setInverted(false);
        rightSlave.setInverted(false);

        leftSlave.equals(leftMaster); //sets slave to follow master
        rightSlave.equals(rightMaster); //sets slave to follow master
    }
}
