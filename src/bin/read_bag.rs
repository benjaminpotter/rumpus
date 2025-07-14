use clap::Parser;
use rosbags_rs::{Reader, TopicInfo};
use std::{fmt, path::PathBuf};
use tabled::{settings::Style, Table, Tabled};

#[derive(Tabled)]
struct TopicRow {
    topic_name: String,
    message_type: String,
    message_count: u64,
}

impl From<TopicInfo> for TopicRow {
    fn from(topic_info: TopicInfo) -> Self {
        Self {
            topic_name: topic_info.name,
            message_type: topic_info.message_type,
            message_count: topic_info.message_count,
        }
    }
}

struct CompressionMode(String);

impl fmt::Display for CompressionMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl From<Option<&str>> for CompressionMode {
    fn from(option: Option<&str>) -> Self {
        match option {
            Some(buf) => Self(buf.to_string()),
            None => Self("none".to_string()),
        }
    }
}

fn main() {
    let args = Args::parse();

    let mut reader = Reader::new(&args.bag).expect("failed to read bag");
    reader.open().unwrap();

    const WIDTH: usize = 20;
    println!("{:WIDTH$} {}", "path", args.bag.display());
    println!("{:WIDTH$} {}", "message_count", reader.message_count());
    println!(
        "{:WIDTH$} {:.2}s",
        "duration",
        reader.duration() as f64 / 1_000_000_000.0
    );

    let metadata = reader.metadata().unwrap();
    println!("{:WIDTH$} {}", "is_compressed", metadata.is_compressed());
    println!(
        "{:WIDTH$} {}",
        "compression_mode",
        CompressionMode::from(metadata.compression_mode())
    );

    let topic_rows = reader.topics().into_iter().map(TopicRow::from);
    let mut topic_table = Table::new(topic_rows);
    topic_table.with(Style::sharp());

    print!("{}", topic_table);
}

#[derive(Parser)]
#[command(about, long_about = None)]
struct Args {
    bag: PathBuf,
}
