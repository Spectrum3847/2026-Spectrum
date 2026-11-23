// Based on 254-2023 Class
// https://github.com/Team254/FRC-2023-Public/blob/main/src/main/java/com/team254/lib/drivers/CanDeviceId.java

/// Identifies a CAN device by its device number and the bus it lives on. Equality and
/// hashing cover both, so the same device number on two buses stays two distinct devices.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct CanDeviceId {
    pub device_number: i32,
    /// The bus name, such as `"rio"` or `"canivore"`, or an empty string for the default bus.
    pub bus: String,
}

impl CanDeviceId {
    pub fn new(device_number: i32, bus: impl Into<String>) -> Self {
        Self {
            device_number,
            bus: bus.into(),
        }
    }

    /// An identifier on the default bus, which is the empty string.
    pub fn on_default_bus(device_number: i32) -> Self {
        Self::new(device_number, "")
    }

    pub fn get_device_number(&self) -> i32 {
        self.device_number
    }

    pub fn get_bus(&self) -> &str {
        &self.bus
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    fn hash_of(id: &CanDeviceId) -> u64 {
        let mut hasher = DefaultHasher::new();
        id.hash(&mut hasher);
        hasher.finish()
    }

    #[test]
    fn constructor_and_getters() {
        let id1 = CanDeviceId::on_default_bus(5);
        assert_eq!(id1.get_device_number(), 5);
        assert_eq!(id1.get_bus(), "");

        let id2 = CanDeviceId::new(10, "rio");
        assert_eq!(id2.get_device_number(), 10);
        assert_eq!(id2.get_bus(), "rio");
    }

    #[test]
    fn equality_and_hash_cover_number_and_bus() {
        let id1 = CanDeviceId::new(1, "canivore");
        let id2 = CanDeviceId::new(1, "canivore");
        let id3 = CanDeviceId::new(1, "rio");
        let id4 = CanDeviceId::new(2, "canivore");

        assert_eq!(id1, id2);
        assert_eq!(hash_of(&id1), hash_of(&id2));
        assert_ne!(id1, id3);
        assert_ne!(id1, id4);
    }
}
