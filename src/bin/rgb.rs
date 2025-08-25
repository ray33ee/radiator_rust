use embassy_time::Instant;

#[derive(Debug)]
pub(crate) struct Color {
    pub(crate) r: u8,
    pub(crate) g: u8,
    pub(crate) b: u8,
}

pub(crate) enum State {
    Off,
    Rainbow(u32),
    Solid(Color),
    Fade(Color, u32), //Color, fade duration
    OneFlash( Color,  u32,  u32), //Color, ont time, off time
    CrossFade{from: Color, to: Color, duration: u32},
    FadeFlash{color: Color, fade_duration: u32, pause: u32, flash_count: u32, on_duration: u32, off_duration: u32, final_pause: u32},

}

#[derive(Debug)]
struct Unit {
    from: Color,
    to: Color,
    duration: u32,
}


impl Unit {
    const fn new(r1: u8, g1: u8, b1: u8, r2: u8, g2: u8, b2: u8, duration: u32) -> Self {
        Self {
            from: Color {
                r: r1,
                g: g1,
                b: b1,
            },
            to: Color {
                r: r2,
                g: g2,
                b: b2,
            },
            duration,
        }
    }

    const fn up_fade(r: u8, g: u8, b: u8, duration: u32) -> Self {
        Self::new(0, 0, 0, r, g, b, duration)
    }

    const fn down_fade(r: u8, g: u8, b: u8, duration: u32) -> Self {
        Self::new(r, g, b, 0, 0, 0, duration)
    }

    const fn solid(r: u8, g: u8, b: u8, duration: u32) -> Self {
        Self::new(r, g, b, r, g, b, duration)
    }

    const fn off(duration: u32) -> Self {
        Self::new(0, 0, 0, 0, 0, 0, duration)
    }

    fn interpolate(&self, x: u32) -> Color {
        let red_gap = self.to.r as i32 - self.from.r as i32;
        let green_gap = self.to.g as i32 - self.from.g as i32;
        let blue_gap = self.to.b as i32 - self.from.b as i32;

        let red = self.from.r as i32 + (red_gap * x as i32) / self.duration as i32;
        let green = self.from.g as i32 + (green_gap * x as i32) / self.duration as i32;
        let blue = self.from.b as i32 + (blue_gap * x as i32) / self.duration as i32;

        Color {
            r: red as u8,
            g: green as u8,
            b: blue as u8,
        }
    }
}


pub(crate) struct RGBLED {
    start_time: Instant,
    units: heapless::Vec<Unit, 20>,
    cycle_duration: u64,
}

impl RGBLED {
    pub(crate) fn new() -> Self {
        Self {
            start_time: Instant::now(),
            units: heapless::Vec::new(),
            cycle_duration: 2,
        }
    }

    pub(crate) fn set_state(&mut self, new_state: State) {
        self.start_time = Instant::now();

        self.units.clear();

        match new_state {
            State::Off => {
                self.units.push(Unit::off(2)).unwrap();
            }
            State::Rainbow(duration) => {
                self.units.push(Unit::new(255, 0, 0, 0, 255, 0, duration / 3)).unwrap();
                self.units.push(Unit::new(0, 255, 0, 0, 0, 255, duration / 3)).unwrap();
                self.units.push(Unit::new(0, 0, 255, 255, 0, 0, duration / 3)).unwrap();
            }
            State::Solid(color) => {
                self.units.push(Unit::solid(color.r, color.g, color.b, 2)).unwrap();
            }
            State::Fade(color, duration) => {
                self.units.push(Unit::up_fade(color.r, color.g, color.b, duration / 2)).unwrap();
                self.units.push(Unit::down_fade(color.r, color.g, color.b, duration / 2)).unwrap();
            }
            State::CrossFade { from, to, duration } => {
                self.units.push(Unit::new(from.r, from.g, from.b, to.r, to.g, to.b, duration / 2)).unwrap();
                self.units.push(Unit::new(to.r, to.g, to.b, from.r, from.g, from.b, duration / 2)).unwrap();
            }
            State::OneFlash(from, on_time, off_time) => {
                self.units.push(Unit::solid(from.r, from.g, from.b, on_time)).unwrap();
                self.units.push(Unit::off(off_time)).unwrap();
            }
            State::FadeFlash { color, fade_duration, pause, flash_count, on_duration, off_duration, final_pause } => {
                self.units.push(Unit::up_fade(color.r, color.g, color.b, fade_duration / 2)).unwrap();
                self.units.push(Unit::down_fade(color.r, color.g, color.b, fade_duration / 2)).unwrap();
                self.units.push(Unit::off(pause)).unwrap();

                for _ in 0..flash_count {
                    self.units.push(Unit::solid(color.r, color.g, color.b, on_duration)).unwrap();
                    self.units.push(Unit::off(off_duration)).unwrap();
                }

                self.units.push(Unit::off(final_pause)).unwrap();

            }
        }

        self.cycle_duration = self.units.iter().map(|x| x.duration as u64 ).sum::<u64>();

    }


    fn interpolate(&self, x: u64) -> Color {

        let x = (x % self.cycle_duration) as u32;

        let mut cumulative = 0;

        for unit in &self.units {
            cumulative += unit.duration;
            if x < cumulative {
                let inner_x = x + unit.duration - cumulative;

                return unit.interpolate(inner_x)
            }
        }

        unreachable!()

    }

    pub(crate) fn update<F: FnOnce(u8, u8, u8) -> ()>(&self, change_color: F) {
        let millis = self.start_time.elapsed().as_millis();

        let color = self.interpolate(millis);

        change_color(color.r, color.g, color.b);

    }
}
