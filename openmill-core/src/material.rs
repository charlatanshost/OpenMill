use serde::{Deserialize, Serialize};

/// A material entry with recommended cutting parameters.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Material {
    /// Display name (e.g. "6061 Aluminum", "HDPE Plastic").
    pub name: String,
    /// Material category for grouping in the UI.
    pub category: MaterialCategory,
    /// Recommended surface speed [m/min] for carbide tooling.
    pub surface_speed: f64,
    /// Recommended chip load per tooth [mm/tooth] for a typical end mill.
    pub chip_load: f64,
    /// Maximum recommended depth of cut as fraction of tool diameter.
    pub max_doc_ratio: f64,
    /// Maximum recommended width of cut as fraction of tool diameter.
    pub max_woc_ratio: f64,
}

/// Broad material category.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MaterialCategory {
    Aluminum,
    Steel,
    Plastic,
    Wood,
    Foam,
    Brass,
    Composite,
}

impl Material {
    /// Compute spindle RPM for a given tool diameter [mm].
    ///
    /// `RPM = (surface_speed × 1000) / (π × diameter)`
    pub fn rpm_for_diameter(&self, diameter: f64) -> f64 {
        (self.surface_speed * 1000.0) / (std::f64::consts::PI * diameter)
    }

    /// Compute feed rate [mm/min] given tool diameter, flute count, and RPM.
    ///
    /// `feed = chip_load × flutes × RPM`
    pub fn feed_rate(&self, flutes: u32, rpm: f64) -> f64 {
        self.chip_load * flutes as f64 * rpm
    }
}

/// A library of materials with default entries for common hobbyist materials.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaterialLibrary {
    pub materials: Vec<Material>,
}

impl Default for MaterialLibrary {
    fn default() -> Self {
        MaterialLibrary {
            materials: vec![
                Material {
                    name: "6061 Aluminum".into(),
                    category: MaterialCategory::Aluminum,
                    surface_speed: 200.0,
                    chip_load: 0.05,
                    max_doc_ratio: 1.0,
                    max_woc_ratio: 0.4,
                },
                Material {
                    name: "7075 Aluminum".into(),
                    category: MaterialCategory::Aluminum,
                    surface_speed: 180.0,
                    chip_load: 0.04,
                    max_doc_ratio: 0.8,
                    max_woc_ratio: 0.35,
                },
                Material {
                    name: "Mild Steel (1018)".into(),
                    category: MaterialCategory::Steel,
                    surface_speed: 80.0,
                    chip_load: 0.03,
                    max_doc_ratio: 0.5,
                    max_woc_ratio: 0.25,
                },
                Material {
                    name: "HDPE Plastic".into(),
                    category: MaterialCategory::Plastic,
                    surface_speed: 300.0,
                    chip_load: 0.10,
                    max_doc_ratio: 2.0,
                    max_woc_ratio: 0.5,
                },
                Material {
                    name: "Acetal (Delrin)".into(),
                    category: MaterialCategory::Plastic,
                    surface_speed: 250.0,
                    chip_load: 0.08,
                    max_doc_ratio: 1.5,
                    max_woc_ratio: 0.5,
                },
                Material {
                    name: "Hardwood (Maple/Oak)".into(),
                    category: MaterialCategory::Wood,
                    surface_speed: 350.0,
                    chip_load: 0.12,
                    max_doc_ratio: 2.0,
                    max_woc_ratio: 0.5,
                },
                Material {
                    name: "Softwood (Pine)".into(),
                    category: MaterialCategory::Wood,
                    surface_speed: 450.0,
                    chip_load: 0.15,
                    max_doc_ratio: 3.0,
                    max_woc_ratio: 0.6,
                },
                Material {
                    name: "Machinable Wax".into(),
                    category: MaterialCategory::Foam,
                    surface_speed: 500.0,
                    chip_load: 0.20,
                    max_doc_ratio: 3.0,
                    max_woc_ratio: 0.8,
                },
                Material {
                    name: "Brass (360)".into(),
                    category: MaterialCategory::Brass,
                    surface_speed: 120.0,
                    chip_load: 0.04,
                    max_doc_ratio: 0.8,
                    max_woc_ratio: 0.3,
                },
            ],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rpm_calculation() {
        let al = &MaterialLibrary::default().materials[0]; // 6061 Aluminum
        let rpm = al.rpm_for_diameter(6.0);
        // 200*1000 / (π*6) ≈ 10610
        assert!((rpm - 10610.0).abs() < 100.0);
    }

    #[test]
    fn feed_calculation() {
        let al = &MaterialLibrary::default().materials[0];
        let rpm = 10000.0;
        let feed = al.feed_rate(2, rpm);
        // 0.05 * 2 * 10000 = 1000
        assert!((feed - 1000.0).abs() < 0.01);
    }

    #[test]
    fn material_library_serde_roundtrip() {
        let lib = MaterialLibrary::default();
        let json = serde_json::to_string(&lib).unwrap();
        let decoded: MaterialLibrary = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.materials.len(), lib.materials.len());
        assert_eq!(decoded.materials[0].name, "6061 Aluminum");
    }
}
