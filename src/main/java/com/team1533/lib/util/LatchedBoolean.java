// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.util;

/**
 * An iterative boolean latch.
 *
 * <p>Returns true once if and only if the value of newValue changes from false to true.
 */
public class LatchedBoolean {
  private boolean mLast = false;

  public boolean update(boolean newValue) {
    boolean ret = false;
    if (newValue && !mLast) {
      ret = true;
    }
    mLast = newValue;
    return ret;
  }
}
