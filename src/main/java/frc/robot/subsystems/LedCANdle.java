// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedCANdle extends SubsystemBase {
  private static CANdle candle = new CANdle(50, "Default Name");
  private static CANdleConfiguration config = new CANdleConfiguration();
  private static final int END_INDEX = 185; //8 for CANdle + 177 for strip

  private static EmptyAnimation empty = new EmptyAnimation(0);
  private static RainbowAnimation rainbow = new RainbowAnimation(0, END_INDEX);
  
  private static Timer timer = new Timer();
  private static int fps = 20;
  private static double[] color = {255, 255, 255};

  // FLAGS
  private static int currentFlag = -1;
  private static String[] flagOptions = {"trans", "bi", "american", "gearcats"};
  private static HashMap<String, RGBWColor[]> flags = new HashMap<String, RGBWColor[]>();
  RGBWColor[] transFlag = {new RGBWColor(45, 103, 175), new RGBWColor(180, 45, 52), new RGBWColor(80, 80, 80), new RGBWColor(180, 45, 52)};
  RGBWColor[] biFlag = {new RGBWColor(180, 45, 52), new RGBWColor(80, 30, 80), new RGBWColor(0, 38, 168)};
  RGBWColor[] americanFlag = {new RGBWColor(150, 0, 0), new RGBWColor(0, 0, 150), new RGBWColor(100, 100, 100)};
  RGBWColor[] gearcatsFlag = {new RGBWColor(2, 92, 80), new RGBWColor(10, 20, 50)};

  /** Creates a new CANdle. */
  public LedCANdle() {
    candle.getConfigurator().apply(config);
    candle.setControl(empty);
    candle.setControl(new SolidColor(0, END_INDEX).withColor(new RGBWColor(0, 255, 0)));

    flags.put("trans", transFlag);
    flags.put("bi", biFlag);
    flags.put("american", americanFlag);
    flags.put("gearcats", gearcatsFlag);

    timer.restart();
  }

  public void setLedColor(int r, int g, int b) {
    candle.setControl(empty);
    candle.setControl(new SolidColor(0, END_INDEX).withColor(new RGBWColor(r, g, b)));
  }

  public void setRainbowAnimation() {
    candle.setControl(empty);
    candle.setControl(rainbow);
  }

  public void colorWithBrightness(DoubleSupplier brightness) {
    // System.out.println(brightness.getAsDouble());
    double scale = brightness.getAsDouble();
    candle.setControl(empty);
    candle.setControl(new SolidColor(0, END_INDEX)
      .withColor(new RGBWColor(
        (int) (color[0] * scale),
        (int) (color[1] * scale),
        (int) (color[2] * scale)
      )
    ));
  }

  public void displayFlag(String flagStr) {
    RGBWColor[] flag = flags.get(flagStr);
    if(flag == null) {
      System.out.println("Flag not found: " + flagStr);
      return;
    }

    int increment = 6;
    int count = 0 + 8; //offset for CANdle
    int color = 0;
    while (count < END_INDEX) {
      candle.setControl(new SolidColor(count, count + increment)
        .withColor(flag[color % flag.length])
      );
      count += increment;
      color++;
    }
  }

  public void cycleFlag() {
    currentFlag = (currentFlag + 1) % flagOptions.length;
    displayFlag(flagOptions[currentFlag]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    try {
      // color = SmartDashboard.getNumberArray("Color {r, g, b}", {255, 255, 255});
    } catch(Exception e) {
      System.out.println("somethings wrong with the led colors womp womp. error: " + e);
    }
    /*
    if((joystickView) && (timer.get() >= (1.0 / fps))) {
      double whiteFactor = ((joystick.getLeftX() * -1) + 1) / 2;
      candle.setControl(new SolidColor(0, 7)
        .withColor(new RGBWColor(
          (int) color[0],
          (int) color[1],
          (int) color[2],
          (int)(whiteFactor * 255))
        )
      );

      timer.restart();
    }
    */
  }
}
