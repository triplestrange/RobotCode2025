// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.util;

// Read a provided input
// When the input becomes true, output true until manually cleared
// Useful for latching
public class StickyBoolean {

  private boolean mOn = false;

  public StickyBoolean() {
    super();
    mOn = false;
  }

  public boolean update(boolean input) {
    mOn |= input;
    return mOn;
  }

  public void reset() {
    mOn = false;
  }

  public boolean get() {
    return mOn;
  }
}
