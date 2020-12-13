/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
  CANSparkMax spark_1 = new CANSparkMax(22, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax spark_2 = new CANSparkMax(23, CANSparkMaxLowLevel.MotorType.kBrushless);
  Encoder encoder = new Encoder(1, 0);

  NetworkTableEntry autoSpeedE = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telem = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

  double[] numberArray = new double[10];

  public Robot() {
      super(.005);
  }

  @Override
  public void autonomousPeriodic() {
      double autospeed = autoSpeedE.getDouble(0.0);
      spark_1.set(-1 * autospeed);
      spark_2.set(autospeed);
    
      
      numberArray[0] = Timer.getFPGATimestamp();
      numberArray[1] = RobotController.getBatteryVoltage();
      numberArray[2] = autospeed;
      numberArray[3] = spark_1.get() * RobotController.getBatteryVoltage();
      numberArray[4] = spark_1.get() * RobotController.getBatteryVoltage();
      numberArray[5] = encoder.getDistance();
      numberArray[6] = encoder.getDistance();
      numberArray[7] = encoder.getRate();
      numberArray[8] = encoder.getRate();
      telem.setDoubleArray(numberArray);

      NetworkTableInstance.getDefault().flush();
  }
}