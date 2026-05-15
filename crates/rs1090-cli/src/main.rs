use clap::Parser;

#[derive(Parser, Debug)]
#[command(name = "rs1090", version, about = "Mode S / ADS-B decoder")]
struct Cli {
    // Subcommands are added with later milestones.
}

// Returns `Result` so subcommands added with later milestones can propagate
// errors without churn at the main entry point.
#[allow(clippy::unnecessary_wraps)]
fn main() -> anyhow::Result<()> {
    let _cli = Cli::parse();
    println!("rs1090 v{} (scaffold)", env!("CARGO_PKG_VERSION"));
    Ok(())
}
