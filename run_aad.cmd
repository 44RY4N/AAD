@echo off
setlocal
pushd "%~dp0" || exit /b 1
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" -m aad_cctv.demo %*
) else (
    python -m aad_cctv.demo %*
)
set "AAD_EXIT=%ERRORLEVEL%"
popd
exit /b %AAD_EXIT%
