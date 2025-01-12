// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.loops;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.List;

/** Interface for registering into the StatusSignalLoop */
public interface IStatusSignalLoop {
  List<BaseStatusSignal> getStatusSignals();

  void onLoop();
}
