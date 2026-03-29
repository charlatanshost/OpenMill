use eframe::egui;

fn main() -> eframe::Result<()> {
    env_logger::init();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("OpenMill — 5-Axis CAM")
            .with_inner_size([1400.0, 900.0]),
        ..Default::default()
    };

    eframe::run_native(
        "OpenMill",
        options,
        Box::new(|_cc| Ok(Box::new(OpenMillApp::default()))),
    )
}

#[derive(Default)]
struct OpenMillApp;

impl eframe::App for OpenMillApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::SidePanel::left("side_panel")
            .resizable(true)
            .default_width(280.0)
            .show(ctx, |_ui| {
                // Side panel — tools, operations, machine config
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.centered_and_justified(|ui| {
                ui.label(
                    egui::RichText::new("OpenMill — 3D Viewport")
                        .size(24.0)
                        .color(egui::Color32::from_gray(160)),
                );
            });
        });
    }
}
