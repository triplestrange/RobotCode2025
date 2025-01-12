// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.util;

public class ToggleBoolean {
  private boolean released = true;
  private boolean retVal = false;

  public boolean update(boolean newValue) {
    if (newValue && released) {
      released = false;
      retVal = !retVal;
    }
    if (!newValue) {
      released = true;
    }
    return retVal;
  }
}
