// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.spectrumLib.util;

public record Trio<A, B, C>(A first, B second, C third) {

    public A getFirst() {
        return first;
    }

    public B getSecond() {
        return second;
    }

    public C getThird() {
        return third;
    }

    public static <A, B, C> Trio<A, B, C> of(A a, B b, C c) {
        return new Trio<>(a, b, c);
    }
}
