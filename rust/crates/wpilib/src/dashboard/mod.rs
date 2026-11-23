//! Dashboard publishing over NetworkTables: SmartDashboard, the Sendable protocol, the
//! chooser, `Field2d`, `Mechanism2d`, `Alert` and the `FMSInfo` table.
//!
//! Dashboard objects hold single-threaded handles. Create and update them on the robot loop
//! thread. [`install`] hooks the per-loop updates into a [`TimedRobot`].

pub mod alert;
pub mod chooser;
pub mod field2d;
pub mod fms_info;
pub mod io;
pub mod mechanism2d;
pub mod sendable;
pub mod smart_dashboard;

use crate::nt::Instance;
use crate::robot::{Robot, TimedRobot};

pub use alert::{Alert, AlertType};
pub use chooser::SendableChooser;
pub use field2d::{Field2d, FieldObject2d};
pub use fms_info::{MatchData, MatchDataSender};
pub use io::{NtIo, SharedIo, TopicIo};
pub use mechanism2d::{Color8Bit, Ligament, Mechanism2d, MechanismLigament2d, MechanismRoot2d};
pub use sendable::{Sendable, SendableBuilder};

/// Runs `SmartDashboard.updateValues()` and publishes `FMSInfo` after `robot_periodic` on
/// every pass. Java publishes `FMSInfo` when it refreshes driver station data, a few steps
/// earlier in the same pass.
///
/// Call after [`nt::start_default_server`](crate::nt::start_default_server), on the robot
/// loop thread. Dashboard use from any other thread then panics.
pub fn install<R: Robot>(robot: &mut TimedRobot<R>) {
    smart_dashboard::claim_loop_thread();
    let mut fms = MatchDataSender::new(NtIo::new(Instance::default_instance()));
    robot.add_post_periodic("FMSInfo", move || fms.refresh());
    robot.add_post_periodic(
        "SmartDashboard.updateValues()",
        smart_dashboard::update_values,
    );
}
