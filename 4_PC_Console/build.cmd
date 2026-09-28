@echo off
rem Build Squid console v2 (Rust GNU toolchain). Adds rustup self-contained dir (dlltool) to PATH.
set "PATH=%USERPROFILE%\.cargo\bin;%USERPROFILE%\.rustup\toolchains\stable-x86_64-pc-windows-gnu\lib\rustlib\x86_64-pc-windows-gnu\bin\self-contained;%PATH%"
cd /d "%~dp0"
cargo build --release || exit /b 1
echo.
echo Built: %~dp0target\release\squid-console2.exe
