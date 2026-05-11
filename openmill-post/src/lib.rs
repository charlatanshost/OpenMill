pub mod fanuc;
pub mod feed_rate;
pub mod grbl;
pub mod linuxcnc;
pub mod traits;

pub use fanuc::FanucPost;
pub use feed_rate::{compute_inverse_time_feed, MAX_ROTARY_VELOCITY};
pub use grbl::GrblPost;
pub use linuxcnc::LinuxCncPost;
pub use traits::PostProcessor;

// Re-export from core for convenience
pub use openmill_core::{PostConfig, Units};

pub const POST_PROCESSOR_NAMES: &[&str] = &["LinuxCNC", "GRBL", "Fanuc"];

pub fn get_post(name: &str) -> Box<dyn PostProcessor> {
    match name {
        "GRBL"  => Box::new(GrblPost),
        "Fanuc" => Box::new(FanucPost),
        _       => Box::new(LinuxCncPost),
    }
}
