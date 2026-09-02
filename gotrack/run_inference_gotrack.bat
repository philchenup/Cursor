@echo off
setlocal EnableExtensions
cd /d "%~dp0"
if not defined HYDRA_FULL_ERROR set "HYDRA_FULL_ERROR=1"
set "PYTHONPATH=%CD%;%PYTHONPATH%"
python -m scripts.inference_gotrack %*
exit /b %ERRORLEVEL%
