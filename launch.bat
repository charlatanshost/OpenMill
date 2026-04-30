@echo off
echo ========================================================
echo                  Starting OpenMill...
echo ========================================================
echo.
echo Building and running in release mode for best performance.
echo (This may take a moment if it's your first time compiling the release profile...)
echo.

cargo run -p openmill-ui --release

if %ERRORLEVEL% neq 0 (
    echo.
    echo OpenMill exited with an error.
    pause
)
