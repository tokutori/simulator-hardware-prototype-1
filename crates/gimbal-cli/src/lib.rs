// SPDX-License-Identifier: MIT

pub mod config;

mod command;
mod generate;
mod manifest;
mod output;
mod validate;

use command::Command;
use std::error::Error;

pub use validate::validate_assembly;

pub fn run() -> Result<(), Box<dyn Error>> {
    let command = Command::parse(std::env::args().nth(1).as_deref())?;
    if command == Command::Help {
        println!("{}", Command::HELP);
        return Ok(());
    }

    let workspace = std::env::current_dir()?;
    let loaded = config::load(
        &workspace.join("parameters.toml"),
        &workspace.join("fabrication.toml"),
    )?;

    match command {
        Command::Generate(mode) => generate::generate(&workspace, &loaded, mode),
        Command::Validate(profile) => validate::validate(&workspace, &loaded, profile),
        Command::RefreshManifest => manifest::refresh_manifest(&workspace),
        Command::CleanOutput => output::clean_output(&workspace),
        Command::Help => unreachable!("help is handled before loading configuration"),
    }
}
