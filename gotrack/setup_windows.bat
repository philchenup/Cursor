@echo off
setlocal EnableExtensions
echo Install PyTorch for your CUDA/CPU build from https://pytorch.org first.
python -m pip install -r "%~dp0requirements-windows.txt"
exit /b %ERRORLEVEL%
