use std::collections::HashMap;

use crate::ini::Ini;

pub const REGISTRIES: [(&str, &str); 4] = [
    ("InfantryTypes", "infantry"),
    ("VehicleTypes", "vehicle"),
    ("AircraftTypes", "aircraft"),
    ("BuildingTypes", "building"),
];

#[derive(Debug, Clone)]
pub struct TypeDef {
    pub kind: String,
    pub index: i32,
    pub name: String,
    pub image: String,
    pub rules: Vec<(String, String)>,
    pub art: Vec<(String, String)>,
}

impl TypeDef {
    pub fn rule(&self, key: &str) -> Option<&str> {
        self.rules
            .iter()
            .find(|(k, _)| k.eq_ignore_ascii_case(key))
            .map(|(_, v)| v.as_str())
    }

    pub fn art_value(&self, key: &str) -> Option<&str> {
        self.art
            .iter()
            .find(|(k, _)| k.eq_ignore_ascii_case(key))
            .map(|(_, v)| v.as_str())
    }

    pub fn speed(&self) -> f64 {
        self.rule("Speed").and_then(|v| v.trim().parse().ok()).unwrap_or(0.0)
    }

    pub fn strength(&self) -> f64 {
        self.rule("Strength").and_then(|v| v.trim().parse().ok()).unwrap_or(0.0)
    }

    pub fn cost(&self) -> f64 {
        self.rule("Cost").and_then(|v| v.trim().parse().ok()).unwrap_or(0.0)
    }

    pub fn is_voxel(&self) -> bool {
        self.art_value("Voxel")
            .map(|v| v.trim().eq_ignore_ascii_case("yes"))
            .unwrap_or(false)
    }
}

#[derive(Debug, Default)]
pub struct TypeTable {
    pub types: Vec<TypeDef>,
    pub unresolved: Vec<String>,
    by_name: HashMap<String, usize>,
}

impl TypeTable {
    pub fn get(&self, name: &str) -> Option<&TypeDef> {
        self.by_name.get(&name.to_ascii_uppercase()).map(|&i| &self.types[i])
    }

    pub fn of_kind(&self, kind: &str) -> Vec<&TypeDef> {
        self.types.iter().filter(|t| t.kind == kind).collect()
    }

    pub fn count_of_kind(&self, kind: &str) -> usize {
        self.types.iter().filter(|t| t.kind == kind).count()
    }
}

pub fn assemble(rules: &Ini, art: &Ini) -> TypeTable {
    let mut table = TypeTable::default();
    for (section, kind) in REGISTRIES {
        let Some(reg) = rules.get(section) else { continue };
        for entry in &reg.entries {
            let Ok(index) = entry.key.trim().parse::<i32>() else { continue };
            let name = entry.value.trim();
            if name.is_empty() {
                continue;
            }
            let Some(def) = rules.get(name) else {
                table.unresolved.push(name.to_string());
                continue;
            };
            let image = def.value("Image").map(|s| s.trim()).unwrap_or(name).to_string();
            let art_entries = art
                .get(&image)
                .map(|s| s.entries.iter().map(|e| (e.key.clone(), e.value.clone())).collect())
                .unwrap_or_default();
            let at = table.types.len();
            table.by_name.insert(name.to_ascii_uppercase(), at);
            table.types.push(TypeDef {
                kind: kind.to_string(),
                index,
                name: name.to_string(),
                image,
                rules: def.entries.iter().map(|e| (e.key.clone(), e.value.clone())).collect(),
                art: art_entries,
            });
        }
    }
    table
}
