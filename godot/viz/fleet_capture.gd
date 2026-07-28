extends Node3D

# Turntable capture of the kytoon fleet.
#
# Loads the five Mk models from ../models/*.glb (exported by
# `python -m kytoon.geometry specs/ -o models`), rigs each one over an
# ocean tethered to a small ship, orbits the camera 360°, and saves a
# PNG per frame.
#
#   godot --path godot fleet.tscn --audio-driver Dummy -- --frames=180 --out=C:/some/dir
#
# The glb exporter authors x-downstream / y-spanwise / z-up; glTF is Y-up,
# so every model gets rotation (-90, 90, 0): up becomes +Y, span lies
# along X, downstream points -Z (wind blows from +Z). Model origins sit
# at the bridle confluence, which is where the tether attaches.

var frames: int = 180
var out_dir: String = "frames"

const ORBIT_RADIUS := 175.0
const LOOK_AT := Vector3(0.0, 25.0, -10.0)

# Entity colors come from KytoonWorld.MK_COLOR, which mirrors kytoon/viz.py
# — fixed per Mk, never re-derived. alt = holder (bridle point) altitude;
# attach_y = keel offset below origin for the Mks whose origin sits inside
# the envelope.
const FLEET := [
	{"model": "mki",   "mk": "I",   "label": "Mk I «Sled»",
	 "x": -90.0, "z": -25.0, "alt": 16.0, "attach_y": 0.0},
	{"model": "mkii",  "mk": "II",  "label": "Mk II «Helikite»",
	 "x": -45.0,  "z": -10.0, "alt": 30.0, "attach_y": -4.9},
	{"model": "mkiii", "mk": "III", "label": "Mk III «Spine»",
	 "x": 0.0,    "z": 0.0,   "alt": 20.0, "attach_y": 0.0},
	{"model": "mkiv",  "mk": "IV",  "label": "Mk IV «Torus»",
	 "x": 45.0,   "z": -10.0, "alt": 32.0, "attach_y": -2.2},
	{"model": "mkv",   "mk": "V",   "label": "Mk V «Manta»",
	 "x": 90.0,   "z": -25.0, "alt": 30.0, "attach_y": -2.9},
]

var cam: Camera3D
var holders: Array[Node3D] = []
var ships: Array[Node3D] = []
var tethers: Array[MeshInstance3D] = []
var anchors: Array[Vector3] = []


func _ready() -> void:
	_parse_args()
	DirAccess.make_dir_recursive_absolute(out_dir)
	_build()

	await get_tree().process_frame
	await RenderingServer.frame_post_draw

	var t_start := Time.get_ticks_msec()
	for i in frames:
		_pose(float(i) / float(frames))
		await get_tree().process_frame
		await RenderingServer.frame_post_draw

		var img: Image = get_viewport().get_texture().get_image()
		var err := img.save_png("%s/frame_%03d.png" % [out_dir, i])
		if err != OK:
			push_error("save failed on frame %d: %d" % [i, err])

		if i == 0:
			print("first frame: %dx%d" % [img.get_width(), img.get_height()])
		if i % 30 == 0 and i > 0:
			print("  %d/%d" % [i, frames])

	var elapsed := (Time.get_ticks_msec() - t_start) / 1000.0
	print("rendered %d frames in %.1fs (%.2fs/frame)" % [
		frames, elapsed, elapsed / float(frames)
	])
	get_tree().quit()


func _parse_args() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--frames="):
			frames = int(arg.split("=")[1])
		elif arg.begins_with("--out="):
			out_dir = arg.split("=")[1]


# --- scene construction --------------------------------------------------

func _build() -> void:
	# tighter fog, shorter shadow throw and a smaller sea than the sim
	# scenes: this is a 175 m turntable, not a 400 m tether
	add_child(KytoonWorld.environment(0.0008))
	KytoonWorld.lights(self, 500.0)
	add_child(KytoonWorld.sea(3000.0))

	cam = Camera3D.new()
	cam.fov = 52.0
	cam.far = 2000.0
	add_child(cam)

	for entry in FLEET:
		_build_kytoon(entry)


func _build_kytoon(entry: Dictionary) -> void:
	var holder := Node3D.new()
	holder.position = Vector3(entry["x"], entry["alt"], entry["z"])
	add_child(holder)
	holders.append(holder)

	# yaw 90: span along X, nose at -Z, so the lineup faces the camera
	var wrapper := KytoonWorld.kite(entry["model"],
		KytoonWorld.MK_COLOR[entry["mk"]], 90.0, 0.05)
	if wrapper == null:
		return
	holder.add_child(wrapper)

	# ship at the tether anchor, upwind (+Z) of the kytoon
	var attach_h: float = entry["alt"] + entry["attach_y"]
	var anchor := Vector3(entry["x"], 0.0, entry["z"] + attach_h * 0.9)
	anchors.append(anchor)
	ships.append(_build_ship(anchor))

	tethers.append(_build_tether())
	_build_label(entry)


func _build_ship(anchor: Vector3) -> Node3D:
	var ship := Node3D.new()
	ship.position = anchor
	add_child(ship)

	var gray := StandardMaterial3D.new()
	gray.albedo_color = Color(0.28, 0.30, 0.34)
	gray.roughness = 0.85

	var hull := MeshInstance3D.new()
	var hull_mesh := BoxMesh.new()
	hull_mesh.size = Vector3(4.0, 2.4, 14.0)
	hull.mesh = hull_mesh
	hull.position = Vector3(0.0, 1.0, 0.0)
	hull.material_override = gray
	ship.add_child(hull)

	var house := MeshInstance3D.new()
	var house_mesh := BoxMesh.new()
	house_mesh.size = Vector3(2.6, 2.0, 3.0)
	house.mesh = house_mesh
	house.position = Vector3(0.0, 3.2, 4.0)
	house.material_override = gray
	ship.add_child(house)

	return ship


func _build_tether() -> MeshInstance3D:
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.82, 0.82, 0.80)
	mat.roughness = 0.9
	var mi := KytoonWorld.line_mesh(0.12, mat)
	add_child(mi)
	return mi


func _build_label(entry: Dictionary) -> void:
	var label := Label3D.new()
	label.text = entry["label"]
	label.font_size = 128
	label.pixel_size = 0.028
	label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	label.modulate = Color(1.0, 1.0, 1.0, 0.85)
	label.outline_size = 22
	label.outline_modulate = Color(0.0, 0.0, 0.0, 0.85)
	label.position = Vector3(entry["x"], 5.0, entry["z"])
	add_child(label)


# --- per-frame pose ------------------------------------------------------

func _pose(t: float) -> void:
	var a := t * TAU

	cam.position = Vector3(
		LOOK_AT.x + cos(a) * ORBIT_RADIUS,
		40.0 + sin(a) * 10.0,
		LOOK_AT.z + sin(a) * ORBIT_RADIUS,
	)
	cam.look_at(LOOK_AT, Vector3.UP)

	for i in holders.size():
		var phase := float(i) * 1.7
		var entry: Dictionary = FLEET[i]

		var heave := sin(a * 2.0 + phase) * 1.5
		holders[i].position.y = entry["alt"] + heave
		holders[i].rotation_degrees.x = sin(a * 2.0 + phase * 2.0) * 2.5

		var bob := sin(a * 3.0 + phase) * 0.4
		ships[i].position.y = bob
		ships[i].rotation_degrees.z = sin(a * 3.0 + phase + 1.0) * 2.0

		var attach := holders[i].global_position + Vector3(0.0, entry["attach_y"], 0.0)
		var deck := anchors[i] + Vector3(0.0, 2.2 + bob, 0.0)
		KytoonWorld.stretch(tethers[i], deck, attach)
