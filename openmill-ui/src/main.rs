mod app;
mod viewport;
mod voxel;

use app::OpenMillApp;

fn main() -> eframe::Result<()> {
    env_logger::init();

    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_title("OpenMill — 5-Axis CAM")
            .with_inner_size([1400.0, 900.0]),
        ..Default::default()
    };

    eframe::run_native(
        "OpenMill",
        options,
        Box::new(|cc| Ok(Box::new(OpenMillApp::new(cc)))),
    )
}
