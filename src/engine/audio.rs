#[derive(Copy, Clone)]
struct AudioChannel {
    enabled: bool,
    panning_left: bool,
    panning_right: bool,
}

impl AudioChannel {
    fn new() -> Self {
        return AudioChannel {
            enabled: false,
            panning_left: false,
            panning_right: false,
        };
    }
}

pub struct Audio {
    enabled: bool,
    channels: [AudioChannel; 4],
    vin_left: bool,
    vin_right: bool,
    volume_left: u8,
    volume_right: u8,
}

impl Audio {
    pub fn new() -> Self {
        return Audio {
            vin_left: false,
            vin_right: false,
            volume_left: 0,
            volume_right: 0,
            enabled: false,
            channels: [AudioChannel::new(); 4],
        };
    }

    /// NR52
    pub fn set_master_control(&mut self, v: u8) {
        self.channels[0].enabled = (v & 0b0001) == 1;
        self.channels[1].enabled = (v & 0b0010) == 1;
        self.channels[2].enabled = (v & 0b0100) == 1;
        self.channels[3].enabled = (v & 0b1000) == 1;
        self.enabled = (v & 0x80) == 1;
    }

    /// NR51
    pub fn set_panning(&mut self, v: u8) {
        self.channels[0].panning_left = (v & 1) == 1;
        self.channels[0].panning_right = (v & 2) == 1;

        self.channels[1].panning_left = (v & 4) == 1;
        self.channels[1].panning_right = (v & 8) == 1;

        self.channels[2].panning_left = (v & 16) == 1;
        self.channels[2].panning_right = (v & 32) == 1;

        self.channels[3].panning_left = (v & 64) == 1;
        self.channels[3].panning_right = (v & 128) == 1;
    }

    /// NR50
    pub fn set_master_volume_and_vin(&mut self, v: u8) {
        self.volume_right = v & 0b00000111;
        self.vin_right = (v & 0b00001000) == 1;
        self.volume_left = v & 0b01110000;
        self.vin_left = (v & 0b00000000) == 1;
    }
}
