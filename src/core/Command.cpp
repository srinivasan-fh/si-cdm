#include "core/Command.hpp"
#include "core/Units.hpp"

#include <cctype>
#include <locale>
#include <sstream>
#include <cmath>

namespace sicdm {

namespace {

constexpr CommandSpec kCommands[] = {
    // Sketch
    {"LINE", "L", "LINE x1,y1 x2,y2", "Line on the active sketch plane (or pick the Line tool)"},
    {"PLINE", "PL", "PLINE x,y x,y ... [CLOSE]", "Polyline through the given points"},
    {"RECT", "REC RECTANG", "RECT x1,y1 x2,y2", "Rectangle from two opposite corners"},
    {"CIRCLE", "C", "CIRCLE cx,cy r [CONSTRUCTION]", "Circle from centre and radius"},
    {"ARC", "A", "ARC start through end | ARC CENTER centre start end [CONSTRUCTION]",
     "Arc through three points, or about a centre counter-clockwise from start to end"},
    {"SPLINE", "SPL", "SPLINE p1 p2 p3 ... [CLOSE] [CONSTRUCTION]", "Smooth curve through the points (closed: a loop you can extrude)"},
    {"ELLIPSE", "EL", "ELLIPSE centre major-axis-end minor-radius [CONSTRUCTION]", "Ellipse from its centre, the end of its major axis and its minor radius"},
    {"SKETCHSLOT", "SSLOT", "SKETCHSLOT centre1 centre2 radius [CONSTRUCTION]", "Sketch slot: two round ends joined by straight sides (SLOT cuts a channel in a body)"},
    {"POLYGON", "POL", "POLYGON centre radius [sides] [CIRCUMSCRIBED] [ANGLE deg] [CONSTRUCTION]",
     "Regular polygon (6 sides by default): corners on the radius, or sides touching it (CIRCUMSCRIBED, across flats)"},
    {"CONSTRUCTION", "CONSTR", "CONSTRUCTION [curve...] [ON|OFF]", "Make sketch curves construction (reference) curves: dashed, not part of profiles"},
    {"EXTEND", "", "EXTEND curve [START|END|AT x,y]", "Carry a line, arc or open polyline end on to the next curve it meets"},
    {"OFFSET", "", "OFFSET curve distance [SIDE x,y]",
     "Copy a sketch curve a distance to one side (default outside / left; negative: inside / right): parallel lines, concentric arcs..."},
    {"DRAG", "", "DRAG curve [START|END|CENTRE|POINT n|RIM] TO x,y [FROM x,y] | DRAG curve BY dx,dy",
     "Drag sketch geometry: the constraints hold, and the curves joined to it follow (as dragging with the mouse)"},
    {"POINT", "PO", "POINT x,y", "Reference point"},
    // Solids
    {"BOX", "BLOCK", "BOX w d h [@x,y,z]", "Block centred on x,y with its base at z"},
    {"CYLINDER", "CYL", "CYLINDER r h [@x,y,z]", "Cylinder along +Z"},
    {"CONE", "", "CONE r h [@x,y,z]", "Cone along +Z"},
    {"SPHERE", "SPH", "SPHERE r [@x,y,z]", "Sphere resting on z"},
    {"TORUS", "TOR", "TORUS R r [@x,y,z]", "Torus around +Z"},
    {"WEDGE", "WE", "WEDGE w d h [@x,y,z]", "Right-angled wedge"},
    {"PYRAMID", "PYR", "PYRAMID r h [sides] [@x,y,z]", "Regular pyramid"},
    {"PRISM", "", "PRISM r h [sides] [@x,y,z]", "Regular prism (default hexagon)"},
    // Features
    {"EXTRUDE", "EXT", "EXTRUDE [curve] distance [SYMMETRIC] [KEEP]",
     "Extrude a closed sketch curve (selected or named) into a solid along its plane normal"},
    // Sculpting & booleans
    {"SCULPT", "SCULPTING BRUSH",
     "SCULPT [body] brush [AT] x,y,z [x,y,z...] [RADIUS r] [STRENGTH 0-1] [INVERT] [SYMMETRY [X|Y|Z...]] [RADIAL n [X|Y|Z]] "
     "[FALLOFF smooth|sphere|sharp|linear|constant] [DETAIL d] [NOREFINE] [MOVE dx,dy,dz]  |  SCULPT [body] REFINE detail  |  SCULPT LIST",
     "Digital clay: DRAW (pull, INVERT pushes), CLAY, CLAYSTRIPS, LAYER, CARVE, INFLATE, BLOB, GRAB (MOVE), SNAKEHOOK, NUDGE, "
     "PINCH, CREASE, SMOOTH, FLATTEN, SCRAPE, FILL, MASK. Dabs along the points every radius / 4; the mesh refines itself"},
    {"REMESH", "DYNAMESH VOXEL", "REMESH [body [\"other\"...]] [VOXEL v] [BLEND b] [SMOOTH s]",
     "Voxel remesh: fuse bodies into one evenly meshed solid, joins filleted to the blend radius, detail finer than s rounded"},
    {"MASK", "", "MASK [body] CLEAR|INVERT|SMOOTH|GROW|SHRINK|ALL [n]",
     "Edit the sculpt mask (the MASK brush paints it; masked surface holds still under the brushes)"},
    {"SUBDIVIDE", "SUBD", "SUBDIVIDE [body] [LEVELS n]", "Loop subdivision: four triangles for each, smoothed"},
    {"SYMMETRIZE", "SYMM", "SYMMETRIZE [body] [X|Y|Z] [-]", "Mirror one side of the body's origin onto the other (+ side by default)"},
    {"DECIMATE", "", "DECIMATE [body] TO n | RATIO r", "Reduce to about n triangles (or the ratio), keeping it closed"},
    {"HOLE", "DRILL", "HOLE [body] [AT] x,y,z [DIR x,y,z] DIAMETER d [DEPTH h | THROUGH]",
     "Drill a hole into the surface at a point (along the surface normal unless DIR); blind holes get a 118 deg tip"},
    {"POCKET", "", "POCKET [body] [AT] x,y,z [DIR x,y,z] [BOX|ROUND|BALL] WIDTH w [LENGTH l] DEPTH d [ANGLE deg]",
     "Cut a rectangular, round or ball-end pocket into the surface"},
    {"CHANNEL", "GROOVE SLOT", "CHANNEL [body] [PATH] x,y,z x,y,z ... WIDTH w [DEPTH d]",
     "Carve a round-bottomed channel following points on the surface (cable runs, vents)"},
    {"TRIM", "KNIFE", "TRIM curve AT x,y | TRIM [body] [PLANE] x,y,z NORMAL x,y,z [SMALLER]",
     "A sketch curve: cut away the piece at x,y between the curves crossing it. A body: slice with a plane, removing the side the normal points to (SMALLER: whichever side is smaller); the cut is capped"},
    {"BOSS", "ADDVOLUME", "BOSS [body] [AT] x,y,z [DIR x,y,z] [CYLINDER|BOX|BALL] DIAMETER d HEIGHT h",
     "Add material: a boss, pad or dome fused onto the surface"},
    {"BOOLEAN", "CSG BOOL", "BOOLEAN body UNION|SUBTRACT|INTERSECT other [KEEP]",
     "Combine two bodies exactly; the other body is used up unless KEEP"},
    {"UNDO", "U", "UNDO", "Undo the last edit: a new, moved or deleted body, a feature, a sculpt or cut, a sketch edit or a CAM change"},
    {"REDO", "", "REDO", "Put back what the last UNDO took away"},
    // Wearable armour & exoskeletons
    {"SHELL", "SOLIDIFY HOLLOW", "SHELL [body] THICKNESS t [OUTWARD] [OPEN x,y,z NORMAL x,y,z]",
     "Hollow a solid to a wall thickness, or solidify a surface; OPEN cuts an opening first (the side the normal points to)"},
    {"ARMOR", "ARMOUR", "ARMOR [body | robot PART word[, word...] [MERGE]] [CLEARANCE d] [THICKNESS t]",
     "Armour shells around a body (or a robot's parts whose names contain the words; MERGE: one shell around them "
     "all): inner face d off the surface, t thick; they move with their parts"},
    {"INTERFERENCE", "CLEARANCE COLLISIONS", "INTERFERENCE [CLEARANCE d] [STEP s] [DURATION s] [BODIES name...]",
     "Sweep the motion timeline and report parts that collide or come closer than the clearance (armour against "
     "the body by default)"},
    {"JOINT", "HINGE PIVOT", "JOINT child [TO parent] AT x,y,z [AXIS X|Y|Z|x,y,z] [HINGE|BALL] [RANGE lo hi] [PERIOD s] "
     "[SIZE d]", "Articulate a part: a hinge pin or ball joint at a point, with a motion swinging it through its range"},
    {"LIGHTEN", "LATTICE LIGHTWEIGHT", "LIGHTEN [body] [SKIN t] [CELL c] [WALL w] [HOLLOW]",
     "Generative lightweighting: keep a skin and fill the inside with a gyroid lattice (or leave it hollow)"},
    // Files
    {"NEW", "", "NEW", "Start an empty project (in the app: a new file tab)"},
    {"OPEN", "", "OPEN path.sicdm", "Open a project (in the app: in its own file tab)"},
    {"RIG", "ROBOTRIG", "RIG [robot] [LIST] [TAG part ROTOR|WHEEL|END_EFFECTOR|FOOT|KNEE|HINGE|BASE|NONE] [MORPHOLOGY m] [BASE FIXED|FLOATING|AUTO] [DENSITY kg/m3] [ENGINE AUTO|ARTICULATED|VEHICLE|OPTIMIZE|PARTICLE] [SIMULATE ON|OFF] [STATE [AT t]] [EXPORT robot.urdf|.sdf|.xml] | RIG ENGINES",
     "A robot read from the parts and their motions: links with mass and inertia, joints, limits and couplings. Tags make wheels and rotors spin, limit knees, mark end effectors and feet"},
    {"TERRAIN", "GROUND", "TERRAIN [OFF | SLOPE | STAIRS | ROUGH | BLOCKS | DEM file.asc|image] [ANGLE deg] [DIRECTION deg] [RISE mm] [RUN mm] [STEPS n] "
     "[AMPLITUDE mm] [WAVELENGTH mm] [HEIGHT mm] [BLOCK mm] [CELL mm] [SIZE x [y]] [AT x,y] [SEED n] [FRICTION mu]",
     "Ground for robots to walk and drive on: a slope, stairs, rough ground, blocks or a digital elevation model (the body \"Terrain\")"},
    {"RECENT", "", "RECENT [n | CLEAR]", "The projects opened or saved lately; RECENT n opens one"},
    {"PREFERENCES", "PREFS SETTINGS", "PREFERENCES [SHOW] | SET units|autosave|material|post|machine|updates|update-url value | RESET",
     "Settings for every project: units for new projects, autosave interval, CAM set-up defaults, update check"},
    {"UPDATES", "UPDATE", "UPDATES [CHECK] [url]", "Is there a newer release? Reads the release feed (the update-url preference, or the address given)"},
    {"LICENCE", "LICENSE", "LICENCE [SHOW] | INSTALL licence.json | REMOVE", "This copy's licence: who it is licensed to, or the Community edition; install or remove a licence file"},
    {"CRASHES", "", "CRASHES [LIST] | SHOW n | CLEAR | FOLDER", "Crash reports kept on this computer (nothing is sent): list, print or delete them"},
    {"PLUGINS", "PLUGIN", "PLUGINS [LIST] | RELOAD | FOLDER", "The plugins found and the commands they add; RELOAD reads them again"},
    {"NOTICES", "", "NOTICES", "The third-party components SI CDM is built with, and their licences"},
    {"RECOVER", "", "RECOVER [n | DISCARD n|ALL]", "Unsaved work kept from a session that ended unexpectedly: list, reopen or discard it"},
    {"SAVE", "", "SAVE [path.sicdm]", "Save the project (to a new path if given)"},
    {"SAVEAS", "", "SAVEAS path.sicdm", "Save the project under a new name"},
    {"FILES", "FILE TABS", "FILES [number | name]", "List the open file tabs, or switch to one"},
    {"CLOSE", "", "CLOSE [DISCARD]", "Close the active file tab (DISCARD drops unsaved changes)"},
    {"IMPORT", "IMP", "IMPORT path [@x,y] [WIDTH mm]",
     "STEP or IGES solids and surfaces (exact; STEP assemblies placed) or STL/OBJ as bodies, DXF onto the sketch plane, PNG/JPG/... as a reference image"},
    {"EXPORT", "EXP", "EXPORT path.step|.igs|.stl|.obj|.3mf|.dxf|.svg|.urdf|.sdf|.xml",
     "STEP or IGES (exact B-reps)/STL/OBJ/3MF of visible bodies; DXF/SVG of the sketch plane; URDF/SDF/MJCF (.xml) of the robot"},
    {"RUN", "SCRIPT", "RUN path.txt", "Run the commands in a text file, one a line (# starts a comment)"},
    // 3D printing
    {"REVOLVE", "REV", "REVOLVE [curve] [ANGLE deg] [AXIS U|V|\"line\"|x,y x,y] [KEEP]",
     "Turn a closed sketch curve about an axis in its plane into a solid (360: closed; less: capped)"},
    {"SWEEP", "", "SWEEP [profile] ALONG path [KEEP]", "Carry a closed profile along a path curve, square to it, mitred at corners"},
    {"LOFT", "", "LOFT \"section\" [AT d] \"section\" [AT d] ... [SMOOTH|RULED] [KEEP]",
     "A solid through closed sections (AT: offset along each section's plane normal)"},
    {"PATTERN", "ARRAY", "PATTERN [body] LINEAR n SPACING d [DIR x,y,z] [ROWS m SPACING2 d DIR2 x,y,z] | CIRCULAR n [ANGLE a] [AXIS X|Y|Z] [AT x,y,z] "
     "[UNION|SUBTRACT \"target\"]", "Linear or circular copies of a body, or joined into / cut from a target"},
    {"MIRROR", "", "MIRROR [body] [PLANE XY|YZ|XZ] [AT x,y,z] [NORMAL x,y,z] [COPY|MOVE] [UNION]", "Mirror a body across a plane"},
    {"FILLET", "ROUND", "FILLET [body] RADIUS r [CONVEX|CONCAVE|BOTH] [VOXEL v] [EDGES ALL|CONVEX|CONCAVE|VERTICAL|HORIZONTAL|TOP|BOTTOM|n...|"
     "NEAR x,y,z...]", "Round a solid's edges with a rolling ball: all of a kind, or exactly on chosen EDGES"},
    {"CHAMFER", "BEVEL", "CHAMFER [body] DISTANCE d [EDGES ALL|CONVEX|CONCAVE|VERTICAL|HORIZONTAL|TOP|BOTTOM|n...|NEAR x,y,z...]",
     "Bevel chosen edges back d along each face (default: the outside edges)"},
    {"EDGES", "", "EDGES [body]", "List a solid's sharp edges, numbered for FILLET / CHAMFER ... EDGES"},
    {"MASSPROPS", "MASS", "MASSPROPS [body] [DENSITY g/cm3 | MATERIAL name]",
     "Volume, area, mass, centre of mass, inertia tensor and principal moments"},
    {"MEASURE", "DIST", "MEASURE x,y,z x,y,z | MEASURE \"body\" \"body\"", "Distance between two points, or the closest distance between two bodies"},
    {"CONSTRAIN", "CON", "CONSTRAIN COINCIDENT|HORIZONTAL|VERTICAL|PARALLEL|PERPENDICULAR|TANGENT|EQUAL|CONCENTRIC|ON|MIDPOINT|FIX curve "
     "[START|END|CENTRE|POINT n|SIDE n] ...", "A geometric constraint between sketch curves; the sketch is solved to meet it"},
    {"DIM", "DIMENSION", "DIM [HORIZONTAL|VERTICAL|DIAMETER|ANGLE] curve [START|END|...] [curve ...] [value]",
     "A driving dimension: a line's length, two points' distance, a circle's radius, two lines' angle"},
    {"CONSTRAINTS", "", "CONSTRAINTS", "The sketch's constraints and dimensions, and its degrees of freedom"},
    {"CONSTRAINT", "", "CONSTRAINT EDIT D1 value | CONSTRAINT DELETE C2",
     "Change a dimension (the model is rebuilt from the history) or remove a constraint"},
    {"HISTORY", "HIST", "HISTORY [LIST] | HISTORY EDIT n command | HISTORY INSERT n command | HISTORY DELETE n | HISTORY CLEAR",
     "The feature history: the model commands in order; edit, insert or delete a step and the model is rebuilt"},
    {"REGEN", "REGENERATE", "REGEN [FORCE]", "Rebuild the model from its feature history"},
    {"CAM", "MILL MACHINE", "CAM SETUP|MACHINE|TOOL|LIBRARY|FACE|ROUGH|POCKET|CONTOUR|PARALLEL|WATERLINE|DRILL|BORE|ROTARY|HOLES|TOOLS|LIST|SIMULATE|POST|CHECK ...",
     "CNC milling: CAM SETUP [body] [STOCK margin|x,y,z] [MATERIAL m] [ORIGIN CORNER|CENTRE|MODEL]; CAM TOOL n FLAT|BALL|BULL|FACE|DRILL|SPOT|"
     "CHAMFER|TAP d [STICKOUT l] [HOLDER d]; CAM LIBRARY [LIST] | IMPORT file.csv|.json | USE #n [AS n] | STORE n|ALL | LOAD file | SAVE file "
     "[CSV|FUSION] (tool libraries, vendor catalogues, Fusion 360); operations take [TOOL n] [STEPDOWN s] [STEPOVER s|%] [LEAVE l] [TOP z] [BOTTOM z|DEPTH d] [ADAPTIVE] [REST] [INDEX a c]...; CAM ROTARY [ALONG] [STEPOVER s] [STEPDOWN d]; CAM 5AXIS [LEADANGLE a] [SIDETILT a] [MAXTILT a]; CAM SWARF [TOOL n] [LEAVE l] [TOP z] [BOTTOM z] [STEPDOWN d] [OUTSIDE|INSIDE|BOTH] (walls with the side of a flat end mill); CAM SIMULATE [SHOW]; "
     "CAM POST file.nc [FANUC|HAAS|GRBL|LINUXCNC] [FORCE] (read back and checked first); CAM CHECK [post | file.nc [post]]; "
     "CAM MACHINE [PRESET vmc|vmc-4th|trunnion|head-table|head-head|benchtop|router] [KINEMATICS table-table|head-table|head-head] [TRAVEL x y z] [ZERO x,y,z] [A min max] [B min max] [C min max] [POCKETS n] [MAXRPM n]"},
    {"LATHE", "TURN TURNING", "LATHE SETUP|TOOL|FACE|ROUGH|FINISH|BORE|GROOVE|DRILL|THREAD|PARTOFF|PROFILE|LIST|SIMULATE|POST ...",
     "CNC turning: LATHE SETUP [body] [AXIS X|Y|Z] [FLIP] [BAR d] [FRONT f] [MATERIAL m] [POST FANUC|HAAS|LINUXCNC] [MAXRPM n]; "
     "LATHE TOOL n TURN [NOSE r] [INSERT C|D|V|T|W] | BORE d | GROOVE w [CORNER r] [REACH l] | THREAD | DRILL d [ANGLE a]; "
     "operations take [TOOL n] [STEPDOWN d] [LEAVE l] [LEAVEZ l] [FROM z] [TO z] [SPEED m/min] [FEED mm/rev] [RPM n]: LATHE FACE; LATHE ROUGH|FINISH "
     "[INSIDE]; LATHE BORE (rough inside); LATHE GROOVE [INSIDE] (the recesses a turning insert or boring bar can't reach); LATHE DRILL [DEPTH d] [PECK p]; "
     "LATHE THREAD PITCH p [FROM z] [TO z] [DIAMETER d] [INSIDE]; LATHE PARTOFF [PECK p]; LATHE SIMULATE [SHOW]; LATHE POST file.nc [post] [CYCLES] (G71/G72 roughing cycles); "
     "Z is 0 at the part's front face, negative towards the chuck"},
    {"SHEET", "DWG", "SHEET NEW|VIEW|SECTION|DIM|AUTODIM|TOL|GDT|DATUM|FINISH|TOLERANCE|TITLE|NOTE|SCALE|SIZE|PROJECTION|EXPORT|SHOW ...",
     "Drawings: SHEET NEW [bodies] [SIZE A3] [PORTRAIT] [THIRD] [SCALE 1:2] lays out front, top, right and isometric views; SHEET VIEW ADD kind [AT x,y]; "
     "SHEET SECTION [view] [VERTICAL|HORIZONTAL] [AT pos] [FLIP]; SHEET DIM HORIZONTAL|VERTICAL|ALIGNED|DIAMETER|RADIUS view x,y,z x,y,z [OFFSET mm]; "
     "SHEET AUTODIM; SHEET TOL n 0.1 | +0.1 -0.05 | LIMITS ... | FIT H7 [VALUES] | BASIC; SHEET GDT position 0.1 DIA MMC DATUMS A B ON view x,y,z | "
     "DIM n; SHEET DATUM A ON view x,y,z; SHEET FINISH RA 1.6 ON view x,y,z; SHEET TOLERANCE ISO 2768-m; SHEET TITLE field text; "
     "SHEET NOTE text [AT x,y]; SHEET EXPORT file.pdf|.svg|.dxf"},
    {"MATE", "MATES", "MATE [type [value] \"A\" AT x,y,z \"B\" AT x,y,z [FLIP]] | FIX body | SOLVE | EDIT | FLIP | SUPPRESS | DELETE | CLEAR",
     "Assembly mates: COINCIDENT, FLUSH, CONCENTRIC, DISTANCE mm, ANGLE deg, PARALLEL, PERPENDICULAR between faces picked by a point on each "
     "(planes, axes of round faces, sphere centres); the second part moves to the first. MATE FIX holds a part still; MATE alone lists them; "
     "MATE EDIT n value, FLIP n, SUPPRESS n, DELETE n; MATE SOLVE places the parts again"},
    {"ASSEMBLY", "ASM", "ASSEMBLY [TREE] | GROUP \"name\" body... | UNGROUP body... | INTERFERENCE [body...] | BOM",
     "The assembly: its tree of sub-assemblies and parts, putting parts in a sub-assembly (a/b for one inside another), the parts that "
     "overlap and by how much, and the parts list"},
    {"PRINTCHECK", "PRINTABILITY", "PRINTCHECK [body] [OVERHANG deg] [WALL mm] [BUILD x,y,z]",
     "Check a part for printing: holes, non-manifold edges, inside out, overhangs, thin walls, size, material, time"},
    {"REPAIR", "FIX", "REPAIR [body]", "Make a mesh printable: weld, drop bad faces, fix the winding, fill holes, drop specks"},
    {"ORIENT", "", "ORIENT [body] [OVERHANG deg]", "Turn a part for the least support and set it on the bed"},
    {"SLICE", "", "SLICE [body] TO path.gcode [LAYER h] [WIDTH w] [INFILL %] [WALLS n] [TOP n] [BOTTOM n] [NOZZLE t] [BED t] "
     "[SPEED v] [FILAMENT d] [RETRACT mm]", "Slice a part for an FDM printer to G-code (walls, infill, solid top and bottom)"},
    // Motion & simulation
    {"MOTION", "MOT MOTIONS", "MOTION CIRCLE|LINEAR [name] [AXIS X|Y|Z|x,y,z] [AT x,y,z] [SPEED v] [RANGE r] "
     "[MODE CONT|LOOP|OSC|ONCE] [START s] [EACH] [FOLLOW circular|NONE] [RATIO r] [RADIUS r] [STEP s] [PATH] "
     "[KEYS t:v,t:v...|NONE] [LIMITS lo hi|OFF] [BODIES name... | ALL]  |  "
     "MOTION DELETE|ON|OFF name  |  MOTION",
     "Circular (deg/s about an axis; EACH: every body about its own centre) or linear (mm/s along a direction) "
     "motion of bodies; FOLLOW makes a linear motion roll with a wheel spin (travel = angle x RADIUS, "
     "default the wheel's size); no args lists them"},
    {"POSE", "POSTURE", "POSE [robot] name | joint angle... [IN s]  |  POSE [robot] JOINTS  |  POSE LIST",
     "Move a library robot (humanoid, arm, hand, leg, neck...) into a posture: a preset (STAND, SQUAT, T_POSE, "
     "HAND_FIST...) and/or joint angles in degrees (RIGHT ELBOW 90, NECK YAW -30); clamped to the joints' "
     "hard stops and reached on minimum-jerk paths within the motors' rated speed"},
    {"GESTURE", "GEST", "GESTURE [robot] name [LOOP|ONCE] [SPEED x|DURATION s] [LEFT|RIGHT|MIRROR]  |  GESTURE LIST",
     "Play a gesture (WAVE, NOD, SHAKE, BOW, POINT, THUMBS_UP, SALUTE, GRASP, COUNT, SQUAT, LOOK_AROUND, "
     "STRETCH, MARCH, WALK) on a library robot from the posture it holds"},
    {"PHYSICS", "PHYS PHYSIC",
     "PHYSICS ON|OFF | DEMO [name] | AT t [body] | GRAVITY OFF|UNIFORM|NEWTON|BOTH [g|x,y,z] [G value] | FLOOR ON|OFF "
     "[FRICTION mu] | AIR ON|OFF | FIELD E|B x,y,z|OFF | AMBIENT degC | RELATIVITY ON|OFF [C m/s] | DURATION s  |  "
     "PHYSICS BODY [name] DYNAMIC|FIXED|OFF [MASS kg] [DENSITY kg/m3] [VELOCITY x,y,z] [SPIN x,y,z] [CHARGE C] "
     "[TEMP degC] [HEAT J/kgK] [BOUNCE e] [FORCE x,y,z] [WAVE SOUND|LIGHT|CUSTOM|OFF] [FREQ Hz] [WAVESPEED v]",
     "Physical environment of the Simulation: gravity (uniform or Newton's between all masses), forces and "
     "collisions, momentum, energy and heat, charges and E / B fields, waves and Doppler, special relativity; "
     "PHYSICS AT t reports kinematics, forces, momentum, energy conservation, temperature, relativity and quantum values"},
    {"SIM", "SIMULATE SIMULATION", "SIM [PLAY|PAUSE|STOP|TIME t|SPEED x|DURATION s|AXES|PATHS ON|OFF|AT t]",
     "Open the Simulation window and control playback; SIM AT t reports body poses at time t"},
    // Library
    {"PART", "INSERT LIBRARY PARTS", "PART name [NAME n] [@x,y,z] [parameter value]...",
     "Insert a library component: motors, gears, racks, gear trains (PART alone lists them)"},
    {"PARAM", "PARAMS PARAMETERS", "PARAM [name] [key value]...",
     "Edit a placed library part: motor shafts (SHAFTS NONE|SINGLE|DUAL, SHAFT_LENGTH...), gear teeth...; no values lists them"},
    {"ROTATE", "ROT", "ROTATE [name] rx,ry,rz", "Orient a body: degrees about X, then Y, then Z, about its origin"},
    {"SCALE", "SC RESIZE", "SCALE [name] factor | SCALE [name] TO size | SCALE [name] sx,sy,sz",
     "Resize a body, or a whole library component with its motions; three factors stretch a body along x, y, z"},
    // Editing
    {"SELECT", "SEL", "SELECT name | NONE", "Select a body, curve or image by name"},
    {"ERASE", "E DEL DELETE", "ERASE [name]", "Delete the selection (or the named item)"},
    {"MOVE", "M", "MOVE dx,dy[,dz] [name]", "Move the selected body or image"},
    {"IMAGE", "IMG", "IMAGE [name] WIDTH mm | OPACITY 0-1 | AT x,y", "Edit a reference image"},
    {"LIST", "LS", "LIST", "List bodies, sketch curves and images"},
    // Views & windows
    {"VIEW", "V", "VIEW TOP|FRONT|RIGHT|BACK|LEFT|BOTTOM|ISO", "Standard view (3D) or drawing plane (2D)"},
    {"2D", "DRAWING", "2D [TOP|FRONT|RIGHT]", "Open the 2D Drawing window"},
    {"3D", "MODEL", "3D", "Open the 3D Model window"},
    {"PLANE", "", "PLANE TOP|FRONT|RIGHT", "Active sketch plane"},
    {"ZOOM", "Z", "ZOOM E|IN|OUT", "Zoom extents (fit) or step in / out"},
    {"FIT", "ZE", "FIT", "Zoom to fit everything"},
    {"HOME", "", "HOME", "Home view: isometric, fit all (keeps the Simulation window; 2D opens 3D)"},
    {"ROLL", "", "ROLL deg | ROLL 0", "Roll the 3D view; 0 levels it"},
    {"ORTHO", "", "ORTHO", "Orthographic projection (3D)"},
    {"PERSP", "PERSPECTIVE", "PERSP", "Perspective projection (3D)"},
    {"DISPLAY", "SHADE", "DISPLAY SHADED|EDGES|WIRE|HIDDEN", "Display mode of the 3D window"},
    {"GRID", "", "GRID [ON|OFF]", "Show or hide the grid"},
    {"SNAP", "", "SNAP [ON|OFF]", "Grid and point snapping for sketches"},
    {"PANEL", "", "PANEL [ON|OFF] | PANEL MODEL|LIBRARY|CAM|TOOLS|MACHINE|GUIDE|PREFERENCES|LICENCE", "Show or hide the docked panel, open it on a page (CAM: milling and lathe jobs), or open the CAM tools, machine, user guide, preferences or licence window"},
    // Session
    {"UNITS", "UN UNIT", "UNITS [MM|CM|M|IN|FT|PT] [PRECISION n|AUTO] [FRACTION 1/n|OFF]",
     "Units for typing and showing lengths (the model stays exact in mm)"},
    {"CONVERT", "CONV", "CONVERT length [TO unit]", "Convert a length, e.g. CONVERT 5'6-1/2\" TO mm"},
    {"HELP", "? COMMANDS", "HELP [command]", "List commands or show one command's usage"},
    {"CLEAR", "CLS", "CLEAR", "Clear the command history"},
};

std::string_view trim(std::string_view s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
    return s;
}

bool iequalsPrefix(std::string_view word, std::string_view prefix) {
    if (prefix.size() > word.size()) return false;
    for (std::size_t i = 0; i < prefix.size(); ++i)
        if (std::toupper(static_cast<unsigned char>(word[i])) != std::toupper(static_cast<unsigned char>(prefix[i])))
            return false;
    return true;
}

bool iequals(std::string_view a, std::string_view b) { return a.size() == b.size() && iequalsPrefix(a, b); }

// Calls f(alias) for each space-separated alias.
template <typename F>
void forEachAlias(const char* aliases, F&& f) {
    std::string_view s = aliases;
    while (!s.empty()) {
        const std::size_t sp = s.find(' ');
        f(s.substr(0, sp));
        if (sp == std::string_view::npos) break;
        s.remove_prefix(sp + 1);
    }
}

} // namespace

std::span<const CommandSpec> commandTable() { return kCommands; }

std::string toUpper(std::string_view s) {
    std::string out(s);
    for (char& c : out) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
}

const CommandSpec* findCommand(std::string_view word) {
    word = trim(word);
    if (word.empty()) return nullptr;
    for (const CommandSpec& c : kCommands) {
        if (iequals(word, c.name)) return &c;
        bool hit = false;
        forEachAlias(c.aliases, [&](std::string_view a) { hit = hit || (!a.empty() && iequals(word, a)); });
        if (hit) return &c;
    }
    return nullptr;
}

std::vector<const CommandSpec*> completeCommand(std::string_view prefix) {
    std::vector<const CommandSpec*> out;
    prefix = trim(prefix);
    if (prefix.empty()) return out;
    for (const CommandSpec& c : kCommands) {
        bool hit = iequalsPrefix(c.name, prefix);
        forEachAlias(c.aliases, [&](std::string_view a) { hit = hit || (!a.empty() && iequalsPrefix(a, prefix)); });
        if (hit) out.push_back(&c);
    }
    return out;
}

ParsedCommand tokenizeCommand(std::string_view line) {
    ParsedCommand pc;
    line = trim(line);
    bool first = true;
    while (!line.empty()) {
        std::string word;
        std::size_t end = 0;
        const bool isQuoted = line.front() == '"';
        if (isQuoted) { // quoted argument (paths and names with spaces)
            end = 1;
            while (end < line.size() && line[end] != '"') {
                if (line[end] == '\\' && end + 1 < line.size() && line[end + 1] == '"') ++end; // \" is a quote
                word += line[end++];
            }
            if (end < line.size()) ++end;
        } else {
            while (end < line.size() && !std::isspace(static_cast<unsigned char>(line[end]))) ++end;
            word = std::string(line.substr(0, end));
        }
        if (first) {
            pc.name = toUpper(word);
        } else {
            pc.args.push_back(std::move(word));
            pc.quoted.push_back(isQuoted);
        }
        first = false;
        line = trim(line.substr(end));
    }
    return pc;
}

std::optional<float> parseNumber(std::string_view text) {
    text = trim(text);
    if (text.empty()) return std::nullopt;
    if (text.front() == '+') text.remove_prefix(1);
    if (text.empty() || text.front() == '+' || text.front() == ' ') return std::nullopt;
    // Floating-point std::from_chars needs macOS 26; parse with the "C" locale instead.
    std::istringstream in{std::string(text)};
    in.imbue(std::locale::classic());
    float v = 0.0f;
    in >> v;
    if (in.fail() || in.peek() != std::char_traits<char>::eof() || !std::isfinite(v)) return std::nullopt;
    return v;
}

std::optional<PointInput> parsePointInput(std::string_view text) {
    text = trim(text);
    PointInput p;
    if (!text.empty() && text.front() == '@') {
        p.relative = true;
        text.remove_prefix(1);
    }
    const std::size_t lt = text.find('<');
    const std::size_t comma = text.find(',');
    if (lt != std::string_view::npos && comma == std::string_view::npos) {
        p.polar = true;
        const auto len = parseLength(text.substr(0, lt));
        const auto ang = parseNumber(text.substr(lt + 1));
        if (!len || !ang) return std::nullopt;
        p.a = static_cast<float>(*len);
        p.b = *ang;
        return p;
    }
    if (comma == std::string_view::npos || text.find(',', comma + 1) != std::string_view::npos) return std::nullopt;
    const auto x = parseLength(text.substr(0, comma));
    const auto y = parseLength(text.substr(comma + 1));
    if (!x || !y) return std::nullopt;
    p.a = static_cast<float>(*x);
    p.b = static_cast<float>(*y);
    return p;
}

Vec2 PointInput::resolve(const Vec2& last) const noexcept {
    Vec2 d{a, b};
    if (polar) {
        const float r = radians(b);
        d = {a * std::cos(r), a * std::sin(r)};
    }
    return relative ? Vec2{last.x + d.x, last.y + d.y} : d;
}

std::optional<Vec3> parseVec3(std::string_view text) {
    text = trim(text);
    if (!text.empty() && text.front() == '@') text.remove_prefix(1);
    float v[3] = {0, 0, 0};
    int n = 0;
    while (true) {
        const std::size_t comma = text.find(',');
        const auto num = parseLength(text.substr(0, comma));
        if (!num || n == 3) return std::nullopt;
        v[n++] = static_cast<float>(*num);
        if (comma == std::string_view::npos) break;
        text.remove_prefix(comma + 1);
    }
    if (n < 2) return std::nullopt;
    return Vec3{v[0], v[1], v[2]};
}

} // namespace sicdm
