// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.spectrumLib.util;

/** Holds three values whose types are independent of each other, for returning them as one. */
public class Trio<A, B, C> {
    private final A m_first;
    private final B m_second;
    private final C m_third;

    public Trio(A first, B second, C third) {
        m_first = first;
        m_second = second;
        m_third = third;
    }

    public A getFirst() {
        return m_first;
    }

    public B getSecond() {
        return m_second;
    }

    public C getThird() {
        return m_third;
    }

    public static <A, B, C> Trio<A, B, C> of(A a, B b, C c) {
        return new Trio<A, B, C>(a, b, c);
    }
}
