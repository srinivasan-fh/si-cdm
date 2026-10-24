"""SPURGEAR teeth module thickness [BORE d]: an SI CDM plugin.

Draws an involute spur gear's outline (20 degree pressure angle) as a closed polyline on the
active sketch plane and extrudes it. It talks to SI CDM over its standard input and output:
one JSON-RPC 2.0 message per line, the same tools an AI agent gets over MCP.
"""

import json
import math
import sys

_next_id = 0


def call(tool, arguments):
    """Runs one SI CDM tool and returns its result (text and whether it failed)."""
    global _next_id
    _next_id += 1
    msg = {"jsonrpc": "2.0", "id": _next_id, "method": "tools/call", "params": {"name": tool, "arguments": arguments}}
    sys.stdout.write(json.dumps(msg) + "\n")
    sys.stdout.flush()
    reply = json.loads(sys.stdin.readline())
    if "error" in reply:
        raise RuntimeError(reply["error"].get("message", "error"))
    result = reply["result"]
    text = "\n".join(c.get("text", "") for c in result.get("content", []))
    if result.get("isError"):
        raise RuntimeError(text)
    return text


def say(text):
    """A line in the command's output."""
    sys.stdout.write(json.dumps({"jsonrpc": "2.0", "method": "sicdm/say", "params": {"text": text}}) + "\n")
    sys.stdout.flush()


def involute(rb, r):
    """The angle along the base circle (rb) where the involute reaches radius r."""
    return math.sqrt(max(r * r - rb * rb, 0.0)) / rb


def outline(teeth, module, pressure=20.0, steps=8):
    rp = module * teeth / 2.0               # pitch circle
    rb = rp * math.cos(math.radians(pressure))  # base circle
    ra = rp + module                        # addendum (tips)
    rf = rp - 1.25 * module                 # dedendum (roots)
    # Half the tooth's angular thickness at the pitch circle, and the involute's angle there.
    half = math.pi / (2.0 * teeth)
    inv_p = math.tan(math.radians(pressure)) - math.radians(pressure)
    pts = []
    for k in range(teeth):
        centre = 2.0 * math.pi * k / teeth
        flank = []
        r0 = max(rb, rf)
        for i in range(steps + 1):
            r = r0 + (ra - r0) * i / steps
            t = involute(rb, r)
            inv = t - math.atan(t)          # involute function at r
            flank.append((r, half + inv_p - inv))
        # Root to tip on one side, tip to root on the other.
        side_a = [(rf, half + inv_p)] if rf < rb else []
        side_a += flank
        for r, a in side_a:
            pts.append((r * math.cos(centre - a), r * math.sin(centre - a)))
        for r, a in reversed(side_a):
            pts.append((r * math.cos(centre + a), r * math.sin(centre + a)))
    return pts, rp, ra, rf


def main(argv):
    args = [a.upper() for a in argv]
    if len(argv) < 3:
        print("SPURGEAR teeth module thickness [BORE d]", file=sys.stderr)
        return 2
    teeth, module, thickness = int(argv[0]), float(argv[1]), float(argv[2])
    bore = float(argv[args.index("BORE") + 1]) if "BORE" in args else 0.0
    if teeth < 6 or module <= 0 or thickness <= 0:
        print("teeth from 6, and a positive module and thickness", file=sys.stderr)
        return 2
    pts, rp, ra, rf = outline(teeth, module)
    coords = " ".join("%.4f,%.4f" % p for p in pts)
    call("run_command", {"command": "PLINE " + coords + " CLOSE"})
    call("run_command", {"command": "EXTRUDE " + repr(thickness)})
    if bore > 0:
        scene = json.loads(call("get_scene", {}))
        gear = scene["bodies"][-1]["name"]
        call("run_command", {"command": 'HOLE "%s" AT 0,0,%s DIAMETER %s THROUGH' % (gear, repr(thickness), repr(bore))})
    say("Spur gear: %d teeth, module %g: pitch %.2f mm, tips %.2f mm, roots %.2f mm across, %g thick%s"
        % (teeth, module, 2 * rp, 2 * ra, 2 * rf, thickness, (", %g bore" % bore) if bore > 0 else ""))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except RuntimeError as e:
        print(e, file=sys.stderr)
        sys.exit(1)
