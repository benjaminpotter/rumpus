use rumpus::sensor::SensorParams;
use serde_json::json;

fn main() {
    let params = SensorParams::default();
    let json = json!(params);
    println!("{}", serde_json::to_string_pretty(&json).unwrap());
}
