"""Train robot controllers on SI CDM's robot engine from Python.

SI CDM simulates a batch of copies of a robot headless and as fast as the machine
runs (the robot_env tool of its MCP server). This module starts the server
(sicdm-mcp), opens a project or builds one from commands, and gives the batch a
vectorised-environment interface:

    from sicdm_env import VecEnv
    env = VecEnv(project="arm.sicdm", count=64, task="reach", target=(300, 200, 300))
    obs = env.reset()                         # (64, observation_size) numpy array
    obs, reward, terminated, truncated, info = env.step(actions)  # actions (64, action_size) in [-1, 1]

The arrays are numpy; hand them to PyTorch with torch.as_tensor(obs) or to JAX with
jax.numpy.asarray(obs), and send actions back the same way (np.asarray(t)).
Copies whose episode ended start again on their own; info["episodes"] lists the
returns of the episodes that ended at that step.

Only the standard library and numpy are needed.
"""

import json
import os
import shutil
import subprocess

import numpy as np

__all__ = ["McpClient", "VecEnv", "find_server"]


def find_server():
    """The sicdm-mcp executable: $SICDM_MCP, next to this file's build, or on PATH."""
    env = os.environ.get("SICDM_MCP")
    if env:
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in ("../build/release/sicdm-mcp", "../build/debug/sicdm-mcp", "../build/sicdm-mcp", "../bin/sicdm-mcp"):
        for ext in ("", ".exe"):
            p = os.path.normpath(os.path.join(here, rel + ext))
            if os.path.isfile(p):
                return p
    p = shutil.which("sicdm-mcp")
    if p:
        return p
    raise FileNotFoundError("sicdm-mcp not found: build SI CDM or set SICDM_MCP to its path")


class McpClient:
    """A sicdm-mcp process spoken to in JSON-RPC over its stdin and stdout."""

    def __init__(self, server=None, project=None):
        args = [server or find_server()]
        if project:
            args.append(project)
        self._proc = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        self._id = 0
        self._request("initialize", {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "sicdm_env", "version": "1"}})
        self._send({"jsonrpc": "2.0", "method": "notifications/initialized"})

    def _send(self, msg):
        self._proc.stdin.write(json.dumps(msg) + "\n")
        self._proc.stdin.flush()

    def _request(self, method, params):
        self._id += 1
        self._send({"jsonrpc": "2.0", "id": self._id, "method": method, "params": params})
        line = self._proc.stdout.readline()
        if not line:
            raise RuntimeError("sicdm-mcp stopped")
        reply = json.loads(line)
        if "error" in reply:
            raise RuntimeError(reply["error"].get("message", "error"))
        return reply["result"]

    def call(self, tool, **arguments):
        """Calls a tool; returns its text (JSON-decoded when it is JSON)."""
        result = self._request("tools/call", {"name": tool, "arguments": arguments})
        text = "\n".join(c.get("text", "") for c in result.get("content", []))
        if result.get("isError"):
            raise RuntimeError(text)
        try:
            return json.loads(text)
        except ValueError:
            return text

    def command(self, line):
        """Runs an SI CDM command line (BOX, JOINT, RIG, PART, TERRAIN ...)."""
        return self.call("run_command", command=line)

    def close(self):
        if self._proc.poll() is None:
            self._proc.stdin.close()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()


class VecEnv:
    """A batch of `count` copies of a robot, stepped together.

    project   a .sicdm file with the robot (or build it with `setup` commands)
    setup     command lines run first, e.g. ["PART robot_arm", "RIG TAG \\"...\\" END_EFFECTOR"]
    robot     which robot, when the scene has more than one
    task      "stand", "walk", "reach" (with target=(x, y, z) in mm), "hover" or "none"
    control   "position" (targets across each joint's range; speeds for wheels and rotors)
              or "torque" (shares of each joint's effort)
    """

    def __init__(self, project=None, setup=(), robot=None, count=1, task="stand", target=None, control="position", control_dt=0.02,
                 episode_seconds=10.0, noise=0.02, seed=1, threads=0, server=None):
        self.client = McpClient(server, project)
        for line in setup:
            self.client.command(line)
        args = dict(op="create", count=int(count), task=task, control=control, control_dt=float(control_dt), episode_seconds=float(episode_seconds),
                    noise=float(noise), seed=int(seed), threads=int(threads))
        if robot is not None:
            args["robot"] = robot
        if target is not None:
            args["target"] = [float(v) for v in target]
        info = self.client.call("robot_env", **args)
        self.id = info["id"]
        self.num_envs = info["count"]
        self.observation_size = info["observation_size"]
        self.action_size = info["action_size"]
        self.observation_names = info["observation_names"]
        self.action_names = info["action_names"]
        self.control_dt = info["control_dt"]
        self._first = self._obs(info["observations"])

    def _obs(self, flat):
        return np.asarray(flat, dtype=np.float32).reshape(self.num_envs, self.observation_size)

    def reset(self, seed=None):
        """Every copy back to the start; returns the observations."""
        r = self.client.call("robot_env", op="reset", id=self.id, seed=int(seed or 0))
        return self._obs(r["observations"])

    def step(self, actions, wrenches=None):
        """Actions (num_envs, action_size) in [-1, 1] -> obs, reward, terminated, truncated, info.

        wrenches: forces your script applies until the next step, a list of dicts
        {"part": name, "force": (fx, fy, fz) N, "torque": (tx, ty, tz) N m,
         "at": (x, y, z) mm on the part at rest, "local": False, "env": copy or -1 for all}.
        """
        a = np.asarray(actions, dtype=np.float64).reshape(self.num_envs, self.action_size)
        args = dict(op="step", id=self.id, actions=a.ravel().tolist())
        if wrenches:
            args["wrenches"] = [{k: (list(map(float, v)) if isinstance(v, (tuple, list, np.ndarray)) else v) for k, v in w.items()} for w in wrenches]
        r = self.client.call("robot_env", **args)
        obs = self._obs(r["observations"])
        reward = np.asarray(r["rewards"], dtype=np.float32)
        terminated = np.asarray(r["terminated"], dtype=bool)
        truncated = np.asarray(r["truncated"], dtype=bool)
        return obs, reward, terminated, truncated, {"episodes": r["episodes"]}

    def close(self):
        try:
            self.client.call("robot_env", op="close", id=self.id)
        finally:
            self.client.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
