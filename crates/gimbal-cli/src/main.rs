// SPDX-License-Identifier: MIT

fn main() {
    if let Err(error) = gimbal_cli::run() {
        eprintln!("error: {error}");
        std::process::exit(1);
    }
}
