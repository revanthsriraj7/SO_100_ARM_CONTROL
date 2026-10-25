# so101-api

Thin Python client for the SO-101 dashboard's REST API. Depends only on
`requests` -- no ROS 2, no PySide6, so it works from any external Python
environment (a plain venv, a Jupyter notebook, another machine on the
network once the dashboard's API is exposed beyond localhost).

## Install

```bash
cd so101_api_client
pip install -e .
```

## Usage

```python
from so101_api import SO101

robot = SO101("http://127.0.0.1:8000")

print(robot.get_state())
print(robot.get_programs())

robot.run_program("BobbinWinding")
```

Or the RoboDK-style nested form:

```python
robot.programs.run("BobbinWinding")
robot.robot.state()
robot.gripper.open()
robot.targets.move("home")
```

See `example_bobbin_winding.py` for a complete monitoring loop, and the
dashboard's own `/docs` (Swagger UI) for the full endpoint reference.
