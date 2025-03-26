// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.util;

import java.util.ArrayList;

/** Helper class for storing and calculating a moving average */
public class MovingAverage {
  ArrayList<Double> numbers = new ArrayList<Double>();
  private int maxSize;

  public MovingAverage(int maxSize) {
    this.maxSize = maxSize;
  }

  public void add(double newNumber) {
    numbers.add(newNumber);
    if (numbers.size() > maxSize) {
      numbers.remove(0);
    }
  }

  public double getAverage() {
    double total = 0;

    for (double number : numbers) {
      total += number;
    }

    return total / numbers.size();
  }

  public int getSize() {
    return numbers.size();
  }

  public boolean isUnderMaxSize() {
    return getSize() < maxSize;
  }

  public void clear() {
    numbers.clear();
  }
}
