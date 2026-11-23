// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#[derive(Debug, Clone, PartialEq)]
pub struct Trio<A, B, C> {
    pub first: A,
    pub second: B,
    pub third: C,
}

impl<A, B, C> Trio<A, B, C> {
    pub fn new(first: A, second: B, third: C) -> Self {
        Self {
            first,
            second,
            third,
        }
    }

    pub fn of(a: A, b: B, c: C) -> Self {
        Self::new(a, b, c)
    }

    pub fn get_first(&self) -> &A {
        &self.first
    }

    pub fn get_second(&self) -> &B {
        &self.second
    }

    pub fn get_third(&self) -> &C {
        &self.third
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constructor_and_getters() {
        let trio = Trio::new("Hello", 123, 4.56_f64);
        assert_eq!(*trio.get_first(), "Hello");
        assert_eq!(*trio.get_second(), 123);
        assert!((*trio.get_third() - 4.56).abs() < 1e-6);
    }

    #[test]
    fn of_builds_the_same_trio() {
        let trio = Trio::of(1, true, "World");
        assert_eq!(*trio.get_first(), 1);
        assert!(*trio.get_second());
        assert_eq!(*trio.get_third(), "World");
        assert_eq!(trio, Trio::new(1, true, "World"));
    }
}
