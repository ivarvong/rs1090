use clap::Parser;

#[derive(Parser, Debug)]
#[command(name = "rs1090", version, about = "Mode S / ADS-B decoder")]
struct Cli {
    // Subcommands are added with later milestones.
}

fn main() -> anyhow::Result<()> {
    let _cli = Cli::parse();
    println!("rs1090 v{} (scaffold)", env!("CARGO_PKG_VERSION"));
    Ok(())
}
