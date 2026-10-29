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
    {"SKETCHFILLET", "SKFILLET", "SKETCHFILLET curve [CORNER n...] radius | SKETCHFILLET line line radius",
     "Round sketch corners: every corner of a polyline or rectangle (or the ones named), or where two lines meet"},
    {"SKETCHCHAMFER", "SKCHAMFER", "SKETCHCHAMFER curve [CORNER n...] distance | SKETCHCHAMFER line line distance",
     "Bevel sketch corners, as SKETCHFILLET"},
    {"SKETCHMIRROR", "SKMIRROR", "SKETCHMIRROR curve... ABOUT line | ABOUT x,y x,y", "Mirrored copies of sketch curves across a line"},
    {"SKETCHPATTERN", "SKPATTERN", "SKETCHPATTERN curve... LINEAR n dx,dy [ROWS m dx,dy] | CIRCULAR n [ABOUT x,y] [ANGLE deg]",
     "Copies of sketch curves in rows or round a point"},
    {"PROJECT", "PROJ", "PROJECT body [EDGES | SECTION] [CONSTRUCTION]",
     "Sketch curves from a body on the sketch plane: its edges seen square to the plane, or where the plane cuts it"},
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
    {"EXTRUDE", "EXT", "EXTRUDE [curve] distance [SYMMETRIC | TWO d2] [REVERSE] [THROUGH] [UPTO NEXT|body|x,y,z] [DRAFT deg] [THIN t] [CUT|JOIN [body]] [KEEP]",
     "Extrude a closed sketch curve (selected or named) into a solid along its plane normal"},
    // Sculpting & booleans
    {"SCULPT", "SCULPTING BRUSH",
     "SCULPT [body] brush [AT] x,y,z [x,y,z...] [RADIUS r] [STRENGTH 0-1] [INVERT] [SYMMETRY [X|Y|Z...]] [RADIAL n [X|Y|Z]] "
     "[FALLOFF smooth|sphere|sharp|linear|constant] [DETAIL d] [NOREFINE] [MOVE dx,dy,dz] [COLOR c] "
     "[ALPHA name|file [ALPHAANGLE deg] [FOLLOW] [TILE n]] [SPACING radii]  |  SCULPT [body] REFINE detail  |  SCULPT LIST | ALPHAS",
     "Digital clay: DRAW (pull, INVERT pushes), CLAY, CLAYSTRIPS, LAYER, CARVE, INFLATE, BLOB, GRAB (MOVE), SNAKEHOOK, NUDGE, "
     "PINCH, CREASE, SMOOTH, FLATTEN, SCRAPE, FILL, MASK, PAINT. Dabs along the points every radius / 4 (SPACING); an ALPHA stamps a pattern; the mesh refines itself"},
    {"REMESH", "DYNAMESH VOXEL", "REMESH [body [\"other\"...]] [VOXEL v] [BLEND b] [SMOOTH s]",
     "Voxel remesh: fuse bodies into one evenly meshed solid, joins filleted to the blend radius, detail finer than s rounded"},
    {"MASK", "", "MASK [body] CLEAR|INVERT|SMOOTH|GROW|SHRINK|ALL [n] | GROUP n|AT x,y,z [INVERT]",
     "Edit the sculpt mask (the MASK brush paints it; masked surface holds still under the brushes), or mask a polygroup"},
    {"PAINT", "POLYPAINT", "PAINT [body] FILL colour [GROUP n|AT x,y,z] | CLEAR",
     "Polypaint: fill a body (or a polygroup) with a colour, the mask holding it off; the PAINT brush paints strokes"},
    {"MATERIAL", "MAT APPEARANCE", "MATERIAL [body...] preset | COLOR c | FINISH polished|satin|brushed|matte | METALLIC m | ROUGHNESS r | TRANSMISSION t | IOR n | EMISSION e | COLORMAP f | NORMALMAP f | DEFAULT | LIST",
     "How a body looks: metals by their measured colour (steel, aluminium, chrome, gold, silver, copper, brass, bronze, titanium, anodised colours, glass...), a finish, or each setting; several named or selected bodies get the same change"},
    {"PROPERTY", "PROP PROPERTIES", "PROPERTY [PROJECT | \"body\"] [LIST] | key value | DELETE key",
     "Product data: the project's or a part's NUMBER, DESCRIPTION, MATERIAL, VENDOR... (the drawing's title block and the parts list use them)"},
    {"REVISION", "REVISIONS", "REVISION | RELEASE | REVIEW | STATE in-work|review|released|obsolete [NOTE text] | NEXT [NOTE text] | SET label | HISTORY",
     "The project's revision (A, B...) and its state: a released revision can't be changed; REVISION NEXT starts the next one"},
    {"COMPARE", "DIFF", "COMPARE file.sicdm | COMPARE first.sicdm second.sicdm | COMPARE REV x [vault item]",
     "What changed between two versions: bodies added, removed, reshaped or moved, parameters, properties, the history, variables"},
    {"VAULT", "PDM", "VAULT [LIST] | FOLDER path | CHECKIN [AS key] [NOTE text] | CHECKOUT key [TO file] | GET key [REV x] [TO file] | UNDO [key] | UNLOCK key FORCE | HISTORY [key]",
     "The team's vault (a shared folder): check projects in and out, so two people don't change one at the same time; every revision kept"},
    {"VIDEO", "TURNTABLE MOVIE", "VIDEO file.avi|.png|.gif|folder/ [TURNTABLE|ANIMATION] [SECONDS s] [FROM t] [TO t] [FPS n] [TURNS k] [ELEVATION deg] [SIZE w h] [SAMPLES n] [QUALITY q] [VIEW iso|front|...|current]",
     "A rendered video: the camera going round the model (turntable), or the motions played (animation); MJPEG AVI, animated PNG, GIF or PNG frames"},
    {"RENDER", "RAYTRACE PHOTO", "RENDER [file.png] [WINDOW|START] [SIZE w h] [SAMPLES n] [BOUNCES b] [ENV studio|sky|white|dark] [SUN az el] [LIGHT x] [EXPOSURE ev] [FLOOR ON|OFF] [DENOISE ON|OFF] [VIEW iso|front|...]",
     "A photoreal picture (path traced) of the 3D view: materials, soft shadows, reflections, glass; without a file, the settings"},
    {"UV", "UVS UNWRAP", "UV [body] UNWRAP [ANGLE deg] [MARGIN m] [NOGROUPS] | CLEAR | INFO | LAYOUT file.png [SIZE n] | SHOW CHECKER | SHOW [COLOR file] [NORMAL file] | HIDE",
     "Texture coordinates: unwrap into flattened, packed charts (polygroups as borders), write the layout, or show a checker or baked maps on the body"},
    {"BAKE", "BAKEMAP", "BAKE [body] NORMAL|OBJECTNORMAL|HEIGHT|OCCLUSION|COLOR [FROM \"source\"] [SIZE n] [RAYS n] [DISTANCE d] [PADDING px] FILE path.png",
     "Bake a texture map onto an unwrapped body: the source's (a sculpt's original) normals, height, colour or ambient occlusion"},
    {"RETOPO", "RETOPOLOGY QUADREMESH ZREMESH", "RETOPO [body] [FACES n] [ITERATIONS k] [NOCURVATURE] [DETAIL levels] [REPLACE]",
     "Retopology: an even all-quad mesh over the body, its edges along the curvature (DETAIL: levels taking the original's detail)"},
    {"MULTIRES", "MULTIRESOLUTION SUBDLEVEL", "MULTIRES [body] ADD [n] | LEVEL n | UP | DOWN | APPLY | DELETE LOWER|HIGHER | LIST",
     "Multiresolution: subdivision levels kept together; sculpt big forms low, detail high, and the detail follows"},
    {"LAYER", "LAYERS", "LAYER [body] NEW name [RECORD] | RECORD name | STOP | STRENGTH name s | ON name | OFF name | DELETE name | BAKE name|ALL | LIST",
     "Sculpt layers: strokes recorded into a layer can be dialled in and out (strength), turned off, deleted or baked"},
    {"POLYGROUP", "POLYGROUPS PGROUP", "POLYGROUP [body] AUTO [ANGLE deg] | ISLANDS | MASK | PAINT | CLEAR | LIST",
     "Polygroups: faces grouped by their angle, connected pieces, the mask or the paint"},
    {"SUBDIVIDE", "SUBD", "SUBDIVIDE [body] [LEVELS n]", "Loop subdivision: four triangles for each, smoothed"},
    {"SYMMETRIZE", "SYMM", "SYMMETRIZE [body] [X|Y|Z] [-]", "Mirror one side of the body's origin onto the other (+ side by default)"},
    {"DECIMATE", "", "DECIMATE [body] TO n | RATIO r", "Reduce to about n triangles (or the ratio), keeping it closed"},
    {"HOLE", "DRILL",
     "HOLE [body] [AT] x,y,z... [DIR x,y,z] DIAMETER d | SIZE M6 [FIT CLOSE|NORMAL|LOOSE] [DEPTH h | THROUGH] [CBORE [d depth] | CSINK [d [deg]]] "
     "[TAPPED [THREAD depth]] [FLAT]  |  HOLE SIZES  |  HOLE LIST [body]",
     "Drill holes into the surface at points (along the surface normal unless DIR): plain, standard sizes, counterbored, countersunk, tapped"},
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
    {"SHELL", "SOLIDIFY HOLLOW", "SHELL [body] THICKNESS t [OUTWARD] [FACE x,y,z | OPEN x,y,z NORMAL x,y,z]",
     "Hollow a solid to a wall thickness, or solidify a surface; FACE leaves that face open, OPEN cuts an opening first (the side the normal points to)"},
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
    {"RIG", "ROBOTRIG", "RIG [robot] [LIST] [TAG part ROTOR|WHEEL|END_EFFECTOR|FOOT|KNEE|HINGE|BASE|NONE] [MORPHOLOGY m] [BASE FIXED|FLOATING|AUTO] [DENSITY kg/m3] [ENGINE AUTO|ARTICULATED|VEHICLE|OPTIMIZE|PARTICLE] [MISSION CLEAR|HOVER|VELOCITY vx,vy,vz|GOTO x,y,z|DRIVE speed [YAW|TURN deg/s] [AT t]] [SIMULATE ON|OFF] [STATE [AT t]] [EXPORT robot.urdf|.sdf|.xml] | RIG ENGINES | "
     "RIG [robot] SENSOR ADD IMU|GPS|FORCE|CAMERA|LIDAR name ON part [AT x,y,z] [LOOK x,y,z] [RATE hz] [NOISE k] [SIZE w h] [FOV deg] [RANGE m] ... | "
     "SENSOR READ name [AT t] | CAPTURE name file [AT t] | LOG name file.csv [FROM t0] [TO t1] | LIST | REMOVE name | "
     "RIG [robot] ENV [COUNT n] [TASK STAND|WALK|REACH|HOVER|NONE] [TARGET x,y,z] [SECONDS s] [TORQUE] [SEED n] | "
     "RIG [robot] FORCE ADD WRENCH|BUOYANCY|MAGNET|SCRIPT name ON part [AT x,y,z] [FORCE fx,fy,fz] [TORQUE tx,ty,tz] [LOCAL] [FROM t] [UNTIL t] "
     "[DENSITY kg/m3] [LEVEL z] [DRAG c] [STRENGTH N] [RANGE d] [ONTO part] [FX \"expr\" ... TZ \"expr\"] [FUNCTION name] | FORCE LIST | FORCE REMOVE name",
     "A robot read from the parts and their motions: links with mass and inertia, joints, limits and couplings. Tags make wheels and rotors spin, limit knees, mark end effectors and feet. "
     "ENV runs a batch of copies headless, fast-forward, for training (python/sicdm_env.py over sicdm-mcp). "
     "FORCE adds forces of any kind to a part: wrenches, buoyancy, magnetic adhesion, scripted expressions"},
    {"ARM", "", "ARM [robot] MOVE_TO x,y,z | IK x,y,z | HOME [TOOL part] [POINT dx,dy,dz] [APPROACH dx,dy,dz] [AT t] [SPEED 0-1] [OPTIMIZE]",
     "Moves a robot arm's tool to a point: inverse kinematics within the joint limits, a collision-free path (the arm, the parts around it and the ground), "
     "timed for the joints' speeds and keyed into their motions. OPTIMIZE (or RIG ENGINE OPTIMIZE) reshapes it for the least effort"},
    {"TERRAIN", "GROUND", "TERRAIN [OFF | SLOPE | STAIRS | ROUGH | BLOCKS | DEM file.asc|image] [ANGLE deg] [DIRECTION deg] [RISE mm] [RUN mm] [STEPS n] "
     "[AMPLITUDE mm] [WAVELENGTH mm] [HEIGHT mm] [BLOCK mm] [CELL mm] [SIZE x [y]] [AT x,y] [SEED n] [FRICTION mu]",
     "Ground for robots to walk and drive on: a slope, stairs, rough ground, blocks or a digital elevation model (the body \"Terrain\")"},
    {"RECENT", "", "RECENT [n | CLEAR]", "The projects opened or saved lately; RECENT n opens one"},
    {"PREFERENCES", "PREFS SETTINGS LANGUAGE", "PREFERENCES [SHOW] | SET units|autosave|material|post|machine|updates|update-url|language value | LANGUAGES | UNTRANSLATED [file.json] | RESET",
     "Settings for every project: units for new projects, autosave interval, CAM set-up defaults, update check, the interface's language"},
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
    {"IMPORT", "IMP", "IMPORT path [@x,y] [WIDTH mm] [UNIT mm|cm|m|inch|foot|micron]",
     "STEP or IGES solids and surfaces (exact; STEP assemblies placed); STL, OBJ, 3MF, AMF, PLY, OFF, glTF/GLB or VRML meshes as bodies (parts, "
     "assemblies, colours; UNIT: one file unit, for files that don't say or say wrong); DXF onto the sketch plane; PNG/JPG/... as a reference image"},
    {"EXPORT", "EXP", "EXPORT path.step|.igs|.stl|.obj|.3mf|.gltf|.glb|.fbx|.dxf|.svg|.urdf|.sdf|.xml",
     "STEP or IGES (exact B-reps)/STL/OBJ/3MF of visible bodies; glTF/GLB/FBX with materials, maps, UVs and skins; DXF/SVG of the sketch "
     "plane; URDF/SDF/MJCF (.xml) of the robot"},
    {"RUN", "SCRIPT", "RUN path.txt", "Run the commands in a text file, one a line (# starts a comment)"},
    // 3D printing
    {"REVOLVE", "REV", "REVOLVE [curve] [ANGLE deg] [AXIS U|V|\"line\"|x,y x,y] [CUT|JOIN [body]] [KEEP]",
     "Turn a closed sketch curve about an axis in its plane into a solid (360: closed; less: capped)"},
    {"SWEEP", "", "SWEEP [profile] ALONG path [KEEP] | SWEEP [profile] HELIX PITCH p TURNS n [AXIS U|V|x,y x,y] [LEFT] [KEEP]",
     "Carry a closed profile along a path curve, square to it, mitred at corners, or round a helix (springs, coils)"},
    {"LOFT", "", "LOFT \"section\" [AT d] \"section\" [AT d] ... [SMOOTH|RULED] [KEEP]",
     "A solid through closed sections (AT: offset along each section's plane normal)"},
    {"PATTERN", "ARRAY", "PATTERN [body] LINEAR n SPACING d [DIR x,y,z] [ROWS m SPACING2 d DIR2 x,y,z] | CIRCULAR n [ANGLE a] [AXIS X|Y|Z] [AT x,y,z] "
     "[UNION|SUBTRACT \"target\"]", "Linear or circular copies of a body, or joined into / cut from a target"},
    {"MIRROR", "", "MIRROR [body] [PLANE XY|YZ|XZ] [AT x,y,z] [NORMAL x,y,z] [COPY|MOVE] [UNION]", "Mirror a body across a plane"},
    {"THREAD", "", "THREAD [body] AT x,y,z AXIS x,y,z SIZE M10 | DIAMETER d PITCH p  LENGTH l [INTERNAL] [LEFT] | THREAD [body] HOLES | HOLE n",
     "Cut a modelled ISO 60 degree thread: on a rod, in a hole, or in the body's tapped wizard holes"},
    {"SHEETMETAL", "SM",
     "SHEETMETAL BASE curve THICKNESS t [RADIUS r] [K k] | SHEETMETAL FLANGE [body] EDGE n | NEAR x,y,z LENGTH l [ANGLE deg] [DOWN] [RADIUS r] | "
     "SHEETMETAL EDGES | FLAT [SKETCH] | INFO [body]",
     "Sheet metal: a base plate, flanges bent on its edges, and the flat pattern (bend allowance from the K factor)"},
    {"FEA", "STRESS",
     "FEA [body] FIX FACE x,y,z | NEAR x,y,z r ... [LOAD FACE x,y,z | NEAR x,y,z r FORCE fx,fy,fz] [PRESSURE FACE x,y,z MPa] [BEARING x,y,z r FORCE f] "
     "[GRAVITY [x,y,z]] [TEMP FACE x,y,z C] [HEAT FACE x,y,z W] [THERMAL] [REF C] [MODES n] [MODE k] [MATERIAL m] [E MPa] [NU v] [YIELD MPa] [CELL size] "
     "[PLOT] [DEFORM scale] | FEA MATERIALS",
     "Analysis: stress (forces, pressures, bearing loads, weight, heat), natural frequencies, heat flow; PLOT colours the body"},
    {"RIB", "", "RIB [curve] THICKNESS t [BODY name] [FLIP] [KEEP]", "A thin wall from an open sketch curve down to the body, centred on the sketch plane"},
    {"EMBOSS", "", "EMBOSS [body] TEXT \"words\" FACE x,y,z HEIGHT h DEPTH d [ANGLE deg] [FONT sans|bold|mono]",
     "Raised lettering on a face, flat or curved (\\n starts a new line)"},
    {"ENGRAVE", "", "ENGRAVE [body] TEXT \"words\" FACE x,y,z HEIGHT h DEPTH d [ANGLE deg] [FONT sans|bold|mono]",
     "Lettering cut into a face, flat or curved"},
    {"SURFACE", "SURF", "SURFACE LOFT curve curve ... | BOUNDARY c1 c2 c3 c4 | EXTRUDE curve DISTANCE d [DIR x,y,z] | REVOLVE curve AXIS x,y,z [AT x,y,z] [ANGLE deg] | "
                        "PLANAR curve | OFFSET body DISTANCE d | BLEND bodyA bodyB [WEIGHT w]",
     "Surfaces (open B-spline bodies): lofted, filled from four curves, extruded, revolved, flat, offset, blended"},
    {"THICKEN", "", "THICKEN body THICKNESS t [BOTH]", "A solid from a surface: offset along its normals, the edges walled"},
    {"KNIT", "SEW STITCH", "KNIT body body ... [TOLERANCE t]", "Stitch surfaces along their shared edges into one; a closed set becomes a solid"},
    {"ANALYZE", "ANALYSE", "ANALYZE ZEBRA [body] | ANALYZE CURVATURE [body] [MEAN|GAUSS] | ANALYZE OFF", "Zebra stripes and curvature colours on a surface"},
    {"DOME", "", "DOME [body] FACE x,y,z HEIGHT h [CONCAVE]", "Raise a dome on a flat face (a quarter ellipse up from its outline), or dish it in with CONCAVE"},
    {"DRAFT", "TAPER", "DRAFT [body] ANGLE deg [PULL x,y,z | X | Y | Z] [NEUTRAL x,y,z]",
     "Tilt a body's side faces (those along the pull) for moulding, about the neutral plane (default: its bottom)"},
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
    {"HISTORY", "HIST", "HISTORY [LIST] | HISTORY EDIT n command | HISTORY SET n KEY value | HISTORY INSERT n command | HISTORY DELETE n | "
                        "HISTORY MOVE n TO m | HISTORY SUPPRESS n | HISTORY UNSUPPRESS n | HISTORY ROLLBACK n | HISTORY ROLLFORWARD | HISTORY CLEAR",
     "The feature history (feature tree): edit, re-parameter, insert, delete, reorder or suppress a step, or roll back, and the model is rebuilt"},
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
    {"COMPONENT", "COMP LINK", "COMPONENT INSERT file [AS name] [AT x,y,z] [ROTATE rx,ry,rz] | LIST | RELOAD [name] [FORCE] | OPEN name | REPLACE name file | MAKE name FILE file FROM body... | BREAK name | MOVE name dx,dy,dz | ROTATE name rx,ry,rz [ABOUT x,y,z] | PACK folder",
     "Assemblies of part files: link a part (or sub-assembly) file in, as often as needed; it updates when the file changes, keeping where it is here"},
    {"FACE", "DIRECT PUSHPULL", "FACE INFO|OFFSET|DELETE|MOVE [body] AT x,y,z [DISTANCE d] [BY dx,dy,dz]",
     "Direct editing of any solid by its faces, picked by a point on them: OFFSET pushes or pulls a face (its neighbours follow; "
     "a round face makes a hole or boss larger or smaller), DELETE takes away a hole, boss, fillet or chamfer, MOVE moves a hole or boss, "
     "INFO says what the face is"},
    {"WELDMENT", "WELD FRAME", "WELDMENT PROFILES [family] | MEMBER profile ALONG curve... | MEMBER profile FROM x,y,z TO x,y,z [TO ...] [ANGLE deg] [ALIGN where] [MIRROR] [UP x,y,z] "
                             "[CORNER MITER|BUTT|NONE] [GROUP name] [MATERIAL m] | JOINT x,y,z MITER|BUTT|THROUGH|SQUARE [member] | EDIT member ... | LIST | CUTLIST [EXPORT file.csv]",
     "Weldments: structural members (SHS, RHS, CHS, FLAT, ROUND, ANGLE, UPE, IPE, custom I and C) along the straight segments of sketch curves, their "
     "ends mitred, butted or cut to the side they meet; the cut list counts members alike with their lengths, angles and masses"},
    {"PATH3D", "3DPATH SKETCH3D 3DSKETCH", "PATH3D x,y,z x,y,z|@dx,dy,dz|refpoint ... [SPLINE] [CLOSE] [BEND r] | POINT curve n x,y,z | INSERT curve n x,y,z | DELETE curve n | "
                                            "BEND curve r | INFO [curve]",
     "3D sketches: a polyline (its corners bent at a radius) or a spline through points in space, for routes, sweeps and weldment frames; "
     "POINT moves a point (@dx,dy,dz: by that much), INSERT adds one before point n, DELETE takes one out, INFO lists them"},
    {"ROUTE", "ROUTING PIPING", "ROUTE [path] PIPE DN20 | PIPE NPS 3/4 [SCH 40|80] | PIPE od x wall | TUBE od x wall | ROD d | CABLE d | HOSE od x wall [BEND r] [MATERIAL m] [NAME n] | "
                                "LIST | BENDS [route] [EXPORT file.csv] | UPDATE [route | ALL] | EDIT route [size] [BEND r] [MATERIAL m] | SIZES [PIPE|TUBE|ROD|CABLE|HOSE]",
     "Routes: pipes, tubes, bars, cables and hoses along a path (a 3D sketch, or a line, polyline, rectangle or spline), exact, bent at the "
     "corners (BEND, else the path's own radius, else 1.5 x OD for pipe, 2 x OD for tube and rod, 6 x OD for cable and hose); BENDS gives the "
     "bend table (feed, rotation, bend angle) and the length to cut; UPDATE refits routes to paths changed since"},
    {"MOLD", "MOULD", "MOLD DRAFT [part] [PULL Z|-Z|x,y,z] [MIN deg] | PARTING [part] [PULL ...] [AT h] | SPLIT [part] [PULL ...] [AT h] [MARGIN m] [SHRINK %] [GAP g] | OFF",
     "Mould tools: DRAFT colours the part by its draft along the pull (green: comes away with the cavity, red: with the core, yellow: too little "
     "draft, purple: undercut); PARTING adds the parting line as closed 3D paths; SPLIT makes the cavity and the core from a block round the part, "
     "flat at the parting height outside it with shut-off walls where the line steps (refused for undercuts)"},
    {"FASTENER", "FASTENERS BOLT SCREW HARDWARE", "FASTENER [LIST] | SIZES type | type size[xlength] [LENGTH l] [AT x,y,z] [DIR x,y,z|-Z...] [NAME n] | "
                                               "INTO [body] [HOLE n,...] [type] [SIZE s] [LENGTH l] [GRIP g] [WASHER] [NUT]",
     "Standard fasteners: hex bolts, socket head cap, countersunk, button head and set screws, hex nuts, plain washers and dowel pins "
     "(ISO metric M1.6 to M24 and ASME inch #2 to 1\"), exact, with their designation (ISO 4762 M6x20) for the parts list. Origin on the "
     "bearing face, shank down DIR. INTO fills the hole wizard's holes: countersunk screws in countersinks, cap screws in counterbores and "
     "tapped holes, hex bolts in plain ones, the length picked from the standard lengths (through the part and a NUT, 1.5 d into a thread); "
     "WASHER adds washers, GRIP leaves room for a part clamped under the head"},
    {"SECTION", "CLIP XSECTION", "SECTION [X|Y|Z|-X|-Y|-Z|PLANE nx,ny,nz] [AT d | AT x,y,z] | MOVE d | FLIP | ON | OFF | CAP ON|OFF | ONLY body... | ALL",
     "Live section view: a plane through the model takes away the material on one side in the 3D window, the cut faces filled and "
     "hatched, so the inside shows (nothing in the model changes). It reports each body's section area and centroid"},
    {"EXPLODE", "EXPLODED", "EXPLODE [LIST] | AUTO [ALONG X|Y|Z|RADIAL] [FACTOR f] [part...] | ADD part... ALONG Z|DIR x,y,z DISTANCE d | EDIT n ... | DELETE n | CLEAR | SHOW | HIDE | TRAILS ON|OFF",
     "Exploded view: steps that move parts (bodies, component instances, sub-assemblies) apart along a direction, one after another. "
     "AUTO stacks the parts apart along an axis (or out from the middle); SHOW and HIDE explode and collapse the 3D view (the parts "
     "stay where they are); drawing views show it with SHEET VIEW ADD ISO EXPLODED"},
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
     "[KEYS t:v,t:v...|NONE] [LIMITS lo hi|OFF] [CONTACT body|NONE [SPRING k] [PRELOAD f] [MASS kg]] [BODIES name... | ALL]  |  "
     "MOTION DELETE|ON|OFF name  |  MOTION",
     "Circular (deg/s about an axis; EACH: every body about its own centre) or linear (mm/s along a direction) "
     "motion of bodies; FOLLOW makes a linear motion roll with a wheel spin (travel = angle x RADIUS, "
     "default the wheel's size); CONTACT makes it ride on a body (a cam, an eccentric, a wedge) as that moves, "
     "pressed against it: a follower's lift along AXIS or a rocker's angle about it (SPRING N/mm, PRELOAD N and "
     "MASS kg give STUDY its contact force); no args lists them"},
    {"POSE", "POSTURE", "POSE [robot] name | joint angle... [IN s]  |  POSE [robot] JOINTS  |  POSE LIST",
     "Move a library robot (humanoid, arm, hand, leg, neck...) into a posture: a preset (STAND, SQUAT, T_POSE, "
     "HAND_FIST...) and/or joint angles in degrees (RIGHT ELBOW 90, NECK YAW -30); clamped to the joints' "
     "hard stops and reached on minimum-jerk paths within the motors' rated speed"},
    {"GESTURE", "GEST", "GESTURE [robot] name [LOOP|ONCE] [SPEED x|DURATION s] [LEFT|RIGHT|MIRROR]  |  GESTURE LIST",
     "Play a gesture (WAVE, NOD, SHAKE, BOW, POINT, THUMBS_UP, SALUTE, GRASP, COUNT, SQUAT, LOOK_AROUND, "
     "STRETCH, MARCH, WALK) on a library robot from the posture it holds"},
    {"PHYSICS", "PHYS PHYSIC",
     "PHYSICS ON|OFF | DEMO [name] | AT t [body] | GRAVITY OFF|UNIFORM|NEWTON|BOTH [g|x,y,z] [G value] | FLOOR ON|OFF "
     "[FRICTION mu] | AIR ON|OFF | WIND x,y,z|OFF (m/s, robots) | FIELD E|B x,y,z|OFF | AMBIENT degC | RELATIVITY ON|OFF [C m/s] | DURATION s  |  "
     "PHYSICS BODY [name] DYNAMIC|FIXED|OFF [MASS kg] [DENSITY kg/m3] [VELOCITY x,y,z] [SPIN x,y,z] [CHARGE C] "
     "[TEMP degC] [HEAT J/kgK] [BOUNCE e] [FORCE x,y,z] [WAVE SOUND|LIGHT|CUSTOM|OFF] [FREQ Hz] [WAVESPEED v]",
     "Physical environment of the Simulation: gravity (uniform or Newton's between all masses), forces and "
     "collisions, momentum, energy and heat, charges and E / B fields, waves and Doppler, special relativity; "
     "PHYSICS AT t reports kinematics, forces, momentum, energy conservation, temperature, relativity and quantum values"},
    {"STUDY", "MOTIONSTUDY PLOT", "STUDY [motion...] [POINT body [x,y,z]]... [FROM t0] [TO t1] [STEP dt | SAMPLES n] [SMOOTH s] [CSV file] [SVG file] [TITLE text]",
     "Motion study: the motions (all of them, or those named) and points on bodies sampled over time: travel or angle, "
     "velocity and acceleration, a point's path, speed and acceleration; a cam follower's contact force on its spring "
     "(MOTION ... CONTACT cam SPRING k PRELOAD f MASS m) and where it would lift off, with the spring it needs. CSV writes "
     "the table, SVG the charts; the Motion Study window plots them"},
    {"INSERTMESH", "IMM", "INSERTMESH [body] SPHERE|CUBE|CYLINDER|CONE|TORUS|BODY \"name\" AT x,y,z [SIZE s] [DIR x,y,z] [SPIN deg] [SINK f] [UNION|APPEND|NEW]",
     "Insert-mesh brush: put a shape (or another body's) on the surface at the point, standing along its normal, SINK of it sunk in; "
     "united exactly (UNION), appended as a polygroup of its own to fuse later with REMESH (APPEND), or a new body (NEW)"},
    {"CURVE", "CURVEBRUSH", "CURVE [body] TUBE PATH x,y,z x,y,z ... [RADIUS r] [SINK f] [UNION|APPEND|NEW] | "
     "CURVE [body] INSERT shape|BODY \"name\" PATH ... [SIZE s] [SPACING d] [SINK f] [UNION|APPEND|NEW]",
     "Curve brushes: a round-ended tube laid along a stroke on the surface, or a row of shapes along it, each facing along the stroke"},
    {"ARMATURE", "ZSPHERE ZSPHERES", "ARMATURE | ARMATURE NEW [name] [AT x,y,z] [RADIUS r] | ARMATURE name ADD [TO n] AT x,y,z [RADIUS r] [MIRROR X|Y|Z] | "
     "INSERT n | MOVE n TO x,y,z | BY dx,dy,dz [CHAIN] | RADIUS n r | REMOVE n | LIST | SHOW | HIDE | SKIN [NAME skin] [VOXEL v] [BLEND b] [SMOOTH k] | "
     "POSE n TURN ax,ay,az | AXIS x,y,z ANGLE a | RESET [MIRROR X|Y|Z] | POSE RESET",
     "Sketch a form as balls joined by tapered links (like ZSpheres): add balls on balls, mirrored across the root's plane; "
     "SKIN fuses them into one smooth closed mesh bound to the armature, and POSE turns a ball and everything on it, bending "
     "the skin (edits to the skin are kept)"},
    {"STACK", "STACKUP TOLSTACK", "STACK | STACK NEW name [NOTE text] [SPEC lo hi | MIN lo | MAX hi] [GAP from to ALONG dir] [SAMPLES n] | "
     "STACK name ADD [label] nominal | BODY body ALONG dir | DIM n [TOL t | TOL upper lower | LIMITS hi lo | FIT code | GENERAL f|m|c|v] "
     "[MINUS] [SENS k] [NORMAL [CP c] | UNIFORM | TRIANGULAR] | STACK name EDIT n ... | REMOVE n | SPEC ... | GAP ... | NOTE text | UPDATE | "
     "LIST | RENAME new | DELETE | STACK name [REPORT] [SAMPLES n] [CSV file] [SVG file]",
     "Tolerance stack-up: a chain of dimensions (typed, a body's size along a direction, or a drawing dimension, with a +- or "
     "two-sided tolerance, an ISO 286 fit or an ISO 2768 general class) adding up to a gap; its range by worst case, RSS, "
     "modified RSS and Monte Carlo, against the spec, with each dimension's share and what to tighten. GAP checks the "
     "chain against the model; UPDATE measures again"},
    {"SIM", "SIMULATE SIMULATION", "SIM [PLAY|PAUSE|STOP|TIME t|SPEED x|DURATION s|AXES|PATHS ON|OFF|AT t] | SIM CHECK [FROM t0] [TO t1] [STEP dt] [ALL] [body...]",
     "Open the Simulation window and control playback; SIM AT t reports body poses at time t; SIM CHECK finds parts that run into each other as it plays"},
    // Library
    {"PART", "INSERT LIBRARY PARTS", "PART name [NAME n] [@x,y,z] [parameter value]...",
     "Insert a library component: motors, gears, racks, gear trains (PART alone lists them)"},
    {"PARAM", "PARAMS PARAMETERS", "PARAM [name] [key value]...",
     "Edit a placed library part: motor shafts (SHAFTS NONE|SINGLE|DUAL, SHAFT_LENGTH...), gear teeth...; no values lists them"},
    {"GIZMO", "MANIPULATOR", "GIZMO MOVE|ROTATE|SCALE|OFF",
     "The Move tool's gizmo on the selected body: arrows and squares move it, rings turn it, cubes scale it (V switches; Ctrl snaps); "
     "each drag is recorded as MOVE, ROTATE ... BY or SCALE ... ABOUT"},
    {"ROTATE", "ROT", "ROTATE [name] rx,ry,rz | ROTATE [name...] BY angle AXIS X|Y|Z|x,y,z [ABOUT x,y,z]",
     "Orient a body: degrees about X, then Y, then Z, about its origin; or turn it on by an angle about an axis through a point "
     "(what the gizmo's rings do); BY turns several named or selected bodies together, about their middle"},
    {"SCALE", "SC RESIZE", "SCALE [name] factor | SCALE [name] TO size | SCALE [name] sx,sy,sz | SCALE [name...] factor|sx,sy,sz ABOUT x,y,z",
     "Resize a body, or a whole library component with its motions; three factors stretch a body along x, y, z; ABOUT scales "
     "bodies about a point (what the gizmo does); several selected scale together about their middle"},
    // Editing
    {"SELECT", "SEL", "SELECT [name... | ADD name... | REMOVE name... | ALL | ALL CURVES | INVERT | NONE]",
     "Select bodies or sketch curves by name (several: the first is the active one), add to or take from the selection, all visible bodies or curves, the others, or none; alone: what is selected"},
    {"ERASE", "E DEL DELETE", "ERASE [name...]", "Delete the selection (every selected body or sketch curve), or the named items"},
    {"RENAME", "REN", "RENAME [old] TO new | RENAME new",
     "Rename a body (the selected one when no old name is given); mates, drawings, CAM, robots, stacks and motions that name it follow"},
    {"HIDE", "", "HIDE [name...]", "Hide the selected bodies (or the named ones); features and exports leave hidden bodies out"},
    {"SHOW", "UNHIDE", "SHOW [name... | ALL]", "Show the named or selected bodies again, or every body"},
    {"ISOLATE", "ISO", "ISOLATE [name...]", "Show only the selected (or named) bodies, hiding all the others; SHOW ALL brings them back"},
    {"COPY", "", "COPY [name...]", "Copy the selected bodies or sketch curves (or the named ones), for PASTE here or in another open file"},
    {"CUT", "", "CUT [name...]", "Copy the selected bodies or sketch curves and take them out (PASTE puts them back in)"},
    {"PASTE", "", "PASTE [OFFSET x,y,z]", "Put in the bodies copied: in place, or beside the originals when they are there (or moved by OFFSET); they are selected. The history keeps them (PASTE CLIP id), so REGEN puts the same bodies in again"},
    {"DUPLICATE", "DUP CLONE", "DUPLICATE [name...] [OFFSET x,y,z]", "Copies of the selected bodies (or the named ones), beside them (or moved by OFFSET); the copies are selected"},
    {"MOVE", "M", "MOVE dx,dy[,dz] [name...]", "Move the selected bodies (all together) or image, or the named bodies"},
    {"IMAGE", "IMG", "IMAGE [name] WIDTH mm | OPACITY 0-1 | AT x,y", "Edit a reference image"},
    {"LIST", "LS", "LIST", "List bodies, sketch curves and images"},
    // Views & windows
    {"VIEW", "V", "VIEW TOP|FRONT|RIGHT|BACK|LEFT|BOTTOM|ISO", "Standard view (3D) or drawing plane (2D)"},
    {"2D", "DRAWING", "2D [TOP|FRONT|RIGHT|datum]", "Open the 2D Drawing window"},
    {"3D", "MODEL", "3D", "Open the 3D Model window"},
    {"REFAXIS", "DATUMAXIS", "REFAXIS name THROUGH p p | AT p DIR x,y,z | PLANES plane plane | OF body | REFAXIS LIST | REFAXIS DELETE name",
     "Reference axes: for circular patterns, revolves and planes (a round body's own axis with OF)"},
    {"REFPOINT", "DATUMPOINT", "REFPOINT name AT x,y,z | CENTRE body | REFPOINT LIST | REFPOINT DELETE name",
     "Reference points: usable wherever a plane or axis takes a point"},
    {"SPLIT", "", "SPLIT [body] BY plane | SPLIT [body] [PLANE] x,y,z NORMAL x,y,z", "Cut a body in two by a plane, keeping both pieces"},
    {"VAR", "VARIABLE EQUATION", "VAR [LIST] | VAR name = expression | VAR DELETE name",
     "Global variables and equations; any command uses them in braces, e.g. BOX {w} {w/2} 10 (changing one rebuilds the model)"},
    {"CONFIG", "CONFIGURATION", "CONFIG [LIST] | CONFIG name | CONFIG SAVE name | CONFIG DELETE name | CONFIG TABLE file.csv",
     "Configurations: named sets of the variables' values; a design table is a CSV of them"},
    {"PLANE", "", "PLANE TOP|FRONT|RIGHT|datum | OFFSET d [FROM p] | ANGLE deg [ABOUT U|V] [FROM p] | THROUGH p1 p2 p3 | AT x,y,z NORMAL n [U u] | FACE x,y,z [ON body]  [NAME n] | LIST | DELETE name", "Sketch plane; datum planes anywhere, on faces"},
    {"ZOOM", "Z", "ZOOM E|IN|OUT", "Zoom extents (fit) or step in / out"},
    {"FIT", "ZE", "FIT", "Zoom to fit everything"},
    {"HOME", "", "HOME", "Home view: isometric, fit all (keeps the Simulation window; 2D opens 3D)"},
    {"ROLL", "", "ROLL deg | ROLL 0", "Roll the 3D view; 0 levels it"},
    {"ORTHO", "", "ORTHO", "Orthographic projection (3D)"},
    {"PERSP", "PERSPECTIVE", "PERSP", "Perspective projection (3D)"},
    {"DISPLAY", "SHADE", "DISPLAY SHADED|EDGES|WIRE|HIDDEN", "Display mode of the 3D window"},
    {"GRID", "", "GRID [ON|OFF]", "Show or hide the grid"},
    {"SNAP", "", "SNAP [ON|OFF]", "Grid and point snapping for sketches"},
    {"PANEL", "", "PANEL [ON|OFF] | PANEL MODEL|LIBRARY|CAM|TOOLS|MACHINE|GUIDE|PREFERENCES|LICENCE|PRODUCT|APPEARANCE|SECTION|PATH3D|STUDY|STACK|ARMATURE", "Show or hide the docked panel, open it on a page (CAM: milling and lathe jobs), or open the CAM tools, machine, user guide, preferences, licence, product data, appearance, section, 3D path and route, motion study, tolerance stack-up or armature window"},
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
