@echo off
rem Run the xplane-bench service in the foreground, logging to service.log.
cd /d "%~dp0..\driver"
python -m xpbench.service --config "%~dp0..\config.local.yaml" >> "%~dp0..\service.log" 2>&1
