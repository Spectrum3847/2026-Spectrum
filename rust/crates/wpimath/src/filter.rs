//! `LinearFilter` from WPILib: FIR and IIR filters.

/// Fixed-size ring that starts zero-filled, like WPILib's `DoubleCircularBuffer`.
#[derive(Clone, Debug)]
struct CircularBuffer {
    data: Vec<f64>,
    front: usize,
}

impl CircularBuffer {
    fn new(size: usize) -> Self {
        Self {
            data: vec![0.0; size],
            front: 0,
        }
    }

    fn add_first(&mut self, value: f64) {
        if self.data.is_empty() {
            return;
        }
        self.front = if self.front == 0 {
            self.data.len() - 1
        } else {
            self.front - 1
        };
        self.data[self.front] = value;
    }

    fn get(&self, index: usize) -> f64 {
        self.data[(self.front + index) % self.data.len()]
    }

    fn clear(&mut self) {
        self.data.fill(0.0);
        self.front = 0;
    }
}

#[derive(Clone, Debug)]
pub struct LinearFilter {
    inputs: CircularBuffer,
    outputs: CircularBuffer,
    input_gains: Vec<f64>,
    output_gains: Vec<f64>,
    last_output: f64,
}

impl LinearFilter {
    pub fn new(ff_gains: &[f64], fb_gains: &[f64]) -> Self {
        Self {
            inputs: CircularBuffer::new(ff_gains.len()),
            outputs: CircularBuffer::new(fb_gains.len()),
            input_gains: ff_gains.to_vec(),
            output_gains: fb_gains.to_vec(),
            last_output: 0.0,
        }
    }

    pub fn single_pole_iir(time_constant: f64, period: f64) -> Self {
        let gain = (-period / time_constant).exp();
        Self::new(&[1.0 - gain], &[-gain])
    }

    pub fn high_pass(time_constant: f64, period: f64) -> Self {
        let gain = (-period / time_constant).exp();
        Self::new(&[gain, -gain], &[-gain])
    }

    /// # Panics
    /// If `taps` is zero.
    pub fn moving_average(taps: usize) -> Self {
        assert!(taps > 0, "Number of taps was not at least 1");
        Self::new(&vec![1.0 / taps as f64; taps], &[])
    }

    pub fn reset(&mut self) {
        self.inputs.clear();
        self.outputs.clear();
    }

    pub fn calculate(&mut self, input: f64) -> f64 {
        let mut ret_val = 0.0;
        if !self.input_gains.is_empty() {
            self.inputs.add_first(input);
        }
        for (i, gain) in self.input_gains.iter().enumerate() {
            ret_val += self.inputs.get(i) * gain;
        }
        for (i, gain) in self.output_gains.iter().enumerate() {
            ret_val -= self.outputs.get(i) * gain;
        }
        if !self.output_gains.is_empty() {
            self.outputs.add_first(ret_val);
        }
        self.last_output = ret_val;
        ret_val
    }

    pub fn last_value(&self) -> f64 {
        self.last_output
    }
}
