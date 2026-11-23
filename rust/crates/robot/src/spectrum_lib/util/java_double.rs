//! Java's `Double.toString`. Java builds command names and alert text by string
//! concatenation, and dashboards and log tools read those strings.

/// Formats like Java: plain decimal from 1e-3 up to 1e7, otherwise `d.dddE<exp>`, and
/// always at least one fractional digit.
pub fn to_string(value: f64) -> String {
    if value.is_nan() {
        return "NaN".to_owned();
    }
    if value.is_infinite() {
        return if value > 0.0 { "Infinity" } else { "-Infinity" }.to_owned();
    }
    if value == 0.0 {
        return if value.is_sign_negative() {
            "-0.0"
        } else {
            "0.0"
        }
        .to_owned();
    }
    let sign = if value < 0.0 { "-" } else { "" };
    let magnitude = value.abs();
    let scientific = format!("{magnitude:e}");
    let (mantissa, exponent) = scientific.split_once('e').unwrap_or((&scientific, "0"));
    let exponent: i32 = exponent.parse().unwrap_or(0);
    let digits: String = mantissa.chars().filter(|c| *c != '.').collect();

    if (1e-3..1e7).contains(&magnitude) {
        let (int_part, frac_part) = if exponent >= 0 {
            let int_len = exponent as usize + 1;
            if digits.len() <= int_len {
                let zeros = "0".repeat(int_len - digits.len());
                (format!("{digits}{zeros}"), "0".to_owned())
            } else {
                (digits[..int_len].to_owned(), digits[int_len..].to_owned())
            }
        } else {
            let zeros = "0".repeat((-exponent - 1) as usize);
            ("0".to_owned(), format!("{zeros}{digits}"))
        };
        format!("{sign}{int_part}.{frac_part}")
    } else {
        let rest = if digits.len() > 1 { &digits[1..] } else { "0" };
        format!("{sign}{}.{rest}E{exponent}", &digits[..1])
    }
}

#[cfg(test)]
mod tests {
    use super::to_string;

    #[test]
    fn whole_numbers_keep_one_fractional_digit() {
        assert_eq!(to_string(5.0), "5.0");
        assert_eq!(to_string(100.0), "100.0");
        assert_eq!(to_string(-2.0), "-2.0");
        assert_eq!(to_string(0.0), "0.0");
        assert_eq!(to_string(-0.0), "-0.0");
    }

    #[test]
    fn fractions_print_shortest_digits() {
        assert_eq!(to_string(0.5), "0.5");
        assert_eq!(to_string(1234.5), "1234.5");
        assert_eq!(to_string(0.001), "0.001");
        assert_eq!(to_string(0.05), "0.05");
        assert_eq!(to_string(-0.25), "-0.25");
        assert_eq!(to_string(9_999_999.0), "9999999.0");
    }

    #[test]
    fn outside_the_plain_range_it_uses_exponent_form() {
        assert_eq!(to_string(1e7), "1.0E7");
        assert_eq!(to_string(12_345_678.9), "1.23456789E7");
        assert_eq!(to_string(1e-4), "1.0E-4");
        assert_eq!(to_string(-1.5e-5), "-1.5E-5");
        assert_eq!(to_string(1e21), "1.0E21");
    }

    #[test]
    fn non_finite_values_use_java_names() {
        assert_eq!(to_string(f64::NAN), "NaN");
        assert_eq!(to_string(f64::INFINITY), "Infinity");
        assert_eq!(to_string(f64::NEG_INFINITY), "-Infinity");
    }
}
