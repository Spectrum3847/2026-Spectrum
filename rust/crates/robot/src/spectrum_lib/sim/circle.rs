//! Draws a circle in a `Mechanism2d` canvas as a ring of evenly spaced radial ligaments.
//! `RollerSim` uses it to show the roller's spin state by color.

use wpilib::dashboard::{Color8Bit, Ligament, Mechanism2d, MechanismLigament2d, MechanismRoot2d};
use wpimath::units::inches_to_meters;

pub struct Circle {
    roller_viz: Option<MechanismLigament2d>,
    circle_background: Vec<MechanismLigament2d>,
    background_lines: i32,
    diameter_inches: f64,
    root: MechanismRoot2d,
    /// Color new background lines get when the circle is drawn.
    pub color: Color8Bit,
    pub name: String,
}

impl Circle {
    pub fn new(
        background_lines: i32,
        diameter_inches: f64,
        name: &str,
        root: &MechanismRoot2d,
        mech: &Mechanism2d,
    ) -> Self {
        Self::with_color(
            mech,
            background_lines,
            diameter_inches,
            name,
            root,
            Color8Bit::BLACK,
        )
    }

    pub fn with_color(
        _mech: &Mechanism2d,
        background_lines: i32,
        diameter_inches: f64,
        name: &str,
        root: &MechanismRoot2d,
        color: Color8Bit,
    ) -> Self {
        let mut circle = Self {
            roller_viz: None,
            circle_background: Vec::new(),
            background_lines,
            diameter_inches,
            root: root.clone(),
            color,
            name: name.to_owned(),
        };
        circle.draw_circle();
        circle
    }

    pub fn circle_background(&self) -> &[MechanismLigament2d] {
        &self.circle_background
    }

    pub fn background_lines(&self) -> i32 {
        self.background_lines
    }

    pub fn draw_circle(&mut self) {
        self.circle_background.clear();
        for i in 0..self.background_lines {
            let line = Ligament::new(
                &format!("{} Background {i}", self.name),
                inches_to_meters(self.diameter_inches) / 2.0,
                (360.0 / f64::from(self.background_lines)) * f64::from(i),
            )
            .with_weight(self.diameter_inches)
            .with_color(self.color);
            self.circle_background.push(self.root.append(line));
        }
    }

    /// Appends a white radius line, so the spin direction is visible in the canvas.
    pub fn draw_viz(&mut self) {
        let line = Ligament::new(
            &format!("{} Roller", self.name),
            inches_to_meters(self.diameter_inches) / 2.0,
            0.0,
        )
        .with_weight(5.0)
        .with_color(Color8Bit::WHITE);
        self.roller_viz = Some(self.root.append(line));
    }

    pub fn set_background_color(&self, color: Color8Bit) {
        for line in &self.circle_background {
            line.set_color(color);
        }
    }

    /// Alternates two colors across the radial lines, which reads as a two-tone circle.
    /// `color` is for even-indexed lines and `color2` for odd-indexed ones.
    pub fn set_half_background(&self, color: Color8Bit, color2: Color8Bit) {
        for (i, line) in self.circle_background.iter().enumerate() {
            line.set_color(if i % 2 == 0 { color } else { color2 });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn circle_initialization_makes_one_line_per_background_line() {
        let mech = Mechanism2d::new(1.0, 1.0);
        let root = mech.get_root("TestRoot", 0.5, 0.5);

        let circle = Circle::new(8, 4.0, "TestCircle", &root, &mech);

        assert_eq!(circle.background_lines(), 8);
        assert_eq!(circle.circle_background().len(), 8);
    }

    #[test]
    fn lines_are_spaced_evenly_with_radius_and_weight_from_the_diameter() {
        let mech = Mechanism2d::new(1.0, 1.0);
        let root = mech.get_root("TestRoot", 0.5, 0.5);
        let circle = Circle::new(8, 4.0, "TestCircle", &root, &mech);
        for (i, line) in circle.circle_background().iter().enumerate() {
            assert!((line.angle() - 45.0 * i as f64).abs() < 1e-12);
            assert!((line.length() - 0.0508).abs() < 1e-12);
            assert_eq!(line.line_weight(), 4.0);
            assert_eq!(line.color(), "#000000");
        }
    }

    #[test]
    fn color_settings_apply_to_every_line() {
        let mech = Mechanism2d::new(1.0, 1.0);
        let root = mech.get_root("TestRoot", 0.5, 0.5);
        let red = Color8Bit::new(255, 0, 0);
        let blue = Color8Bit::new(0, 0, 255);

        let circle = Circle::with_color(&mech, 4, 4.0, "TestCircle", &root, red);

        let colors = |c: &Circle| -> Vec<String> {
            c.circle_background().iter().map(|l| l.color()).collect()
        };
        assert_eq!(colors(&circle), ["#FF0000"; 4]);

        circle.set_background_color(blue);
        assert_eq!(colors(&circle), ["#0000FF"; 4]);

        circle.set_half_background(red, blue);
        assert_eq!(
            colors(&circle),
            ["#FF0000", "#0000FF", "#FF0000", "#0000FF"]
        );
    }

    #[test]
    fn draw_viz_appends_a_white_radius_line() {
        let mech = Mechanism2d::new(1.0, 1.0);
        let root = mech.get_root("TestRoot", 0.5, 0.5);
        let mut circle = Circle::new(2, 4.0, "C", &root, &mech);
        circle.draw_viz();
        let viz = circle.roller_viz.as_ref().expect("viz");
        assert_eq!(viz.color(), "#FFFFFF");
        assert_eq!(viz.line_weight(), 5.0);
        assert_eq!(viz.angle(), 0.0);
    }
}
