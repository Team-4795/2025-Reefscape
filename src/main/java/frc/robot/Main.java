// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.text.ParseException;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main  {
  private Main() {}

  public String Bruh = "Bruh";

  public static void main(String... args) {
    RobotBase.startRobot(() -> {
      try {
        return new Robot();
      } catch (IOException | org.json.simple.parser.ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
            return null;
    });
  }
}
