// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class OSUtil {
  public static void fsSync() {
    try {
      System.out.println("Executing filesystem sync...");
      ProcessBuilder builder = new ProcessBuilder("sync");
      builder.redirectErrorStream(true);
      Process p = builder.start();
      BufferedReader r = new BufferedReader(new InputStreamReader(p.getInputStream()));
      String line;
      while (true) {
        line = r.readLine();
        if (line == null) {
          break;
        }
      }
      System.out.println("Done");
    } catch (IOException e) {
      DriverStation.reportError(
          "Failed to manually execute filesystem 'sync' command to flush logs to disk", null);
    }
  }

  public static void fsSyncAsync() {
    new Thread(
            () -> {
              fsSync();
            })
        .start();
  }
}
