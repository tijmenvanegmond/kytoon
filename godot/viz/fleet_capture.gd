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

# Entity colors from kytoon/viz.py MK_COLOR — fixed per Mk, never re-derived.
# alt = holder (bridle point) altitude; attach_y = keel offset below origin
# for the Mks whose origin sits inside the envelope.
const FLEET := [
	{"file": "mki.glb",   "label": "Mk I «Sled»",     "color": Color("2a78d6"),
	 "x": -90.0, "z": -25.0, "alt": 16.0, "attach_y": 0.0},
	{"file": "mkii.glb",  "label": "Mk II «Helikite»", "color": Color("1baf7a"),
	 "x": -45.0,  "z": -10.0, "alt": 30.0, "attach_y": -4.9},
	{"file": "mkiii.glb", "label": "Mk III «Spine»",   "color": Color("eda100"),
	 "x": 0.0,    "z": 0.0,   "alt": 20.0, "attach_y": 0.0},
	{"file": "mkiv.glb",  "label": "Mk IV «Torus»",    "color": Color("008300"),
	 "x": 45.0,   "z": -10.0, "alt": 32.0, "attach_y": -2.2},
	{"file": "mkv.glb",   "label": "Mk V «Manta»",     "color": Color("4a3aa7"),
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
	_build_environment()
	_build_lights()
	_build_sea()

	cam = Camera3D.new()
	cam.fov = 52.0
	cam.far = 2000.0
	add_child(cam)

	var models_dir := ProjectSettings.globalize_path("res://") + "../models/"
	for entry in FLEET:
		_build_kytoon(entry, models_dir)


func _build_environment() -> void:
	var sky_mat := ProceduralSkyMaterial.new()
	sky_mat.sky_top_color = Color(0.13, 0.24, 0.42)
	sky_mat.sky_horizon_color = Color(0.78, 0.62, 0.45)
	sky_mat.ground_horizon_color = Color(0.55, 0.45, 0.38)
	sky_mat.ground_bottom_color = Color(0.05, 0.08, 0.12)
	var sky := Sky.new()
	sky.sky_material = sky_mat

	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.ambient_light_energy = 0.5
	env.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	env.tonemap_exposure = 1.0
	env.fog_enabled = true
	env.fog_light_color = Color(0.55, 0.58, 0.65)
	env.fog_density = 0.0008

	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)


func _build_lights() -> void:
	var sun := DirectionalLight3D.new()
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 500.0
	sun.light_energy = 1.8
	sun.light_color = Color(1.0, 0.93, 0.82)
	sun.rotation_degrees = Vector3(-24.0, 35.0, 0.0)
	add_child(sun)

	var fill := DirectionalLight3D.new()
	fill.shadow_enabled = false
	fill.light_energy = 0.35
	fill.light_color = Color(0.6, 0.7, 1.0)
	fill.rotation_degrees = Vector3(-18.0, -140.0, 0.0)
	add_child(fill)


func _build_sea() -> void:
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.03, 0.10, 0.17)
	mat.metallic = 0.05
	mat.metallic_specular = 0.08
	mat.roughness = 0.6
	var plane := PlaneMesh.new()
	plane.size = Vector2(3000.0, 3000.0)
	var mi := MeshInstance3D.new()
	mi.mesh = plane
	mi.material_override = mat
	add_child(mi)


func _build_kytoon(entry: Dictionary, models_dir: String) -> void:
	var holder := Node3D.new()
	holder.position = Vector3(entry["x"], entry["alt"], entry["z"])
	add_child(holder)
	holders.append(holder)

	var inner := Node3D.new()
	inner.rotation_degrees = Vector3(-90.0, 90.0, 0.0)
	holder.add_child(inner)

	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	var err := doc.append_from_file(models_dir + entry["file"], state)
	if err != OK:
		push_error("failed to load %s: %d" % [entry["file"], err])
		return
	var model := doc.generate_scene(state)
	inner.add_child(model)

	var mat := StandardMaterial3D.new()
	mat.albedo_color = entry["color"]
	mat.roughness = 0.55
	mat.metallic = 0.05
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED   # canopies are open surfaces
	_apply_material(model, mat)

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
	var mesh := CylinderMesh.new()
	mesh.top_radius = 0.12
	mesh.bottom_radius = 0.12
	mesh.height = 1.0
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	mi.material_override = mat
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


func _apply_material(node: Node, mat: Material) -> void:
	if node is MeshInstance3D:
		node.material_override = mat
	for child in node.get_children():
		_apply_material(child, mat)


# --- per-frame pose ------------------------------------------------------

func _stretch_tether(t: MeshInstance3D, a: Vector3, b: Vector3) -> void:
	var d := b - a
	(t.mesh as CylinderMesh).height = d.length()
	var y := d.normalized()
	var x := y.cross(Vector3.FORWARD).normalized()
	if x.length_squared() < 0.5:
		x = y.cross(Vector3.RIGHT).normalized()
	var z := x.cross(y)
	t.global_transform = Transform3D(Basis(x, y, z), (a + b) * 0.5)


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
		_stretch_tether(tethers[i], deck, attach)
