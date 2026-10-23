@echo off
rem Windows release from a normal Command Prompt or PowerShell: runs scripts/release.sh in
rem MSYS2's UCRT64 environment. Arguments pass through, e.g.  scripts\release.cmd --clean
rem MSYS2 lives in C:\msys64 unless MSYS2_ROOT says otherwise.
setlocal
if "%MSYS2_ROOT%"=="" set "MSYS2_ROOT=C:\msys64"
if not exist "%MSYS2_ROOT%\usr\bin\bash.exe" (
    echo MSYS2 not found in %MSYS2_ROOT%. Install it from https://www.msys2.org, then in its UCRT64 shell:
    echo   pacman -S git mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja mingw-w64-ucrt-x86_64-nsis
    exit /b 1
)
set MSYSTEM=UCRT64
set CHERE_INVOKING=1
"%MSYS2_ROOT%\usr\bin\bash.exe" -lc "scripts/release.sh windows %*"
