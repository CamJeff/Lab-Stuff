bin/arduino-cli.exe compile --fqbn arduino:avr:uno control_lab/control_lab.ino && \
bin/arduino-cli.exe upload --port COM3 --fqbn arduino:avr:uno control_lab/control_lab.ino && \
uv run orchestrator.py
