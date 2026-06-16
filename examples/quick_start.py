from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from differential_drive_sandbox.controllers import PurePursuitController
from differential_drive_sandbox.paths import straight_line
from differential_drive_sandbox.robot import DifferentialDriveRobot
from differential_drive_sandbox.simulation import SimulationConfig, SimulationEngine


path = straight_line(length=3.0)
robot = DifferentialDriveRobot()
controller = PurePursuitController(path=path, target_speed=0.35)
engine = SimulationEngine(robot, SimulationConfig(duration=10.0, dt=0.05))

samples = engine.run(controller)
print(f"final pose: {robot.get_pose()}")
print(f"samples: {len(samples)}")
