extends Node3D

# Replays a Mk V longitudinal sim trajectory (CSV from
# scratchpad/export_mkv_replay.py via kytoon.solvers.l1_trim dynamics).
#
#   godot --path godot mkv_replay.tscn --audio-driver Dummy -- \
#       --csv=C:/path/mkv_replay.csv --out=C:/path/frames
#
# Mapping sim -> scene: sim x (downwind) -> +X, sim z (up) -> +Y,
# longitudinal plane at Z = 0. Pitch theta (nose-up) -> rotation.z = -theta.
# Line endpoints and the winchlet pod come straight from the CSV — no
# geometry re-derived here. CSV columns:
# t,U,alpha,theta,x,z, xm,zm (main attach), xc,zc (ctl attach),
# xp,zp (pod), T_main,T_ctl

var csv_path := "renders/mkv_replay.csv"
var out_dir := "frames"

const FAIRLEAD := Vector3(0.0, 5.0, 0.0)
const MK_V_COLOR := Color("4a3aa7")

var rows: Array = []
var kite: Node3D
var pod: MeshInstance3D
var cam: Camera3D
var hud: Label
var line_main: MeshInstance3D
var line_ctl_l: MeshInstance3D
var line_ctl_r: MeshInstance3D
var mat_main: StandardMaterial3D
var mat_ctl: StandardMaterial3D

const Y_CTL := 12.16     # outboard station spanwise offset (0.88 span pos)


func _ready() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--csv="):
			csv_path = arg.split("=")[1]
		elif arg.begins_with("--out="):
			out_dir = arg.split("=")[1]
	_load_csv()
	DirAccess.make_dir_recursive_absolute(out_dir)
	_build()

	await get_tree().process_frame
	await RenderingServer.frame_post_draw

	var t_start := Time.get_ticks_msec()
	for i in rows.size():
		_pose(i)
		await get_tree().process_frame
		await RenderingServer.frame_post_draw
		var img: Image = get_viewport().get_texture().get_image()
		img.save_png("%s/frame_%04d.png" % [out_dir, i])
		if i % 100 == 0:
			print("  %d/%d" % [i, rows.size()])
	var el := (Time.get_ticks_msec() - t_start) / 1000.0
	print("rendered %d frames in %.1fs" % [rows.size(), el])
	get_tree().quit()


func _load_csv() -> void:
	var f := FileAccess.open(csv_path, FileAccess.READ)
	if f == null:
		push_error("cannot open " + csv_path)
		get_tree().quit()
		return
	f.get_csv_line()                      # header
	while not f.eof_reached():
		var cells := f.get_csv_line()
		if cells.size() >= 14:
			var r: Array[float] = []
			for c in cells:
				r.append(float(c))
			rows.append(r)
	print("loaded %d rows" % rows.size())


# --- scene ----------------------------------------------------------------

func _build() -> void:
	# environment (fleet-scene family)
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
	env.fog_enabled = true
	env.fog_light_color = Color(0.55, 0.58, 0.65)
	env.fog_density = 0.0004
	var we := WorldEnvironment.new()
	we.environment = env
	add_child(we)

	var sun := DirectionalLight3D.new()
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 800.0
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

	# sea
	var sea := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(20000.0, 20000.0)
	sea.mesh = plane
	var smat := StandardMaterial3D.new()
	smat.albedo_color = Color(0.03, 0.10, 0.17)
	smat.metallic = 0.05
	smat.metallic_specular = 0.08
	smat.roughness = 0.6
	sea.material_override = smat
	add_child(sea)

	_build_ship()

	# kite
	kite = Node3D.new()
	add_child(kite)
	var inner := Node3D.new()
	inner.rotation_degrees = Vector3(-90.0, 0.0, 0.0)   # x downwind, z -> up
	kite.add_child(inner)
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	if doc.append_from_file(ProjectSettings.globalize_path("res://")
			+ "../models/mkv.glb", state) == OK:
		var model := doc.generate_scene(state)
		inner.add_child(model)
		var mat := StandardMaterial3D.new()
		mat.albedo_color = MK_V_COLOR
		mat.roughness = 0.55
		mat.cull_mode = BaseMaterial3D.CULL_DISABLED
		_apply(model, mat)

	# lines
	mat_main = StandardMaterial3D.new()
	mat_main.roughness = 0.9
	mat_ctl = StandardMaterial3D.new()
	mat_ctl.roughness = 0.9
	line_main = _make_line(0.20, mat_main)
	line_ctl_l = _make_line(0.10, mat_ctl)
	line_ctl_r = _make_line(0.10, mat_ctl)

	# winchlet pod riding the main line
	pod = MeshInstance3D.new()
	var pm := CapsuleMesh.new()
	pm.radius = 0.45
	pm.height = 1.8
	pod.mesh = pm
	var pmat := StandardMaterial3D.new()
	pmat.albedo_color = Color(0.9, 0.55, 0.1)
	pmat.roughness = 0.5
	pod.material_override = pmat
	add_child(pod)

	cam = Camera3D.new()
	cam.fov = 50.0
	cam.far = 5000.0
	add_child(cam)

	var canvas := CanvasLayer.new()
	add_child(canvas)
	hud = Label.new()
	hud.position = Vector2(24, 18)
	hud.add_theme_font_size_override("font_size", 22)
	hud.add_theme_color_override("font_color", Color(1, 1, 1, 0.95))
	hud.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.9))
	hud.add_theme_constant_override("outline_size", 8)
	canvas.add_child(hud)


func _build_ship() -> void:
	var gray := StandardMaterial3D.new()
	gray.albedo_color = Color(0.28, 0.30, 0.34)
	gray.roughness = 0.85
	var ship := Node3D.new()
	add_child(ship)
	# 40 m trimaran: center hull + outriggers + deckhouse + winch pedestal
	for def in [[Vector3(40, 3.5, 4), Vector3(0, 1.5, 0)],
				[Vector3(20, 2, 1.6), Vector3(-4, 1.0, 8)],
				[Vector3(20, 2, 1.6), Vector3(-4, 1.0, -8)],
				[Vector3(6, 3, 5), Vector3(-12, 4.5, 0)],
				[Vector3(2, 2.2, 2), Vector3(0, 4.1, 0)]]:
		var b := MeshInstance3D.new()
		var m := BoxMesh.new()
		m.size = def[0]
		b.mesh = m
		b.position = def[1]
		b.material_override = gray
		ship.add_child(b)


func _make_line(radius: float, mat: StandardMaterial3D) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var mesh := CylinderMesh.new()
	mesh.top_radius = radius
	mesh.bottom_radius = radius
	mesh.height = 1.0
	mi.mesh = mesh
	mi.material_override = mat
	add_child(mi)
	return mi


func _apply(n: Node, mat: Material) -> void:
	if n is MeshInstance3D:
		n.material_override = mat
	for c in n.get_children():
		_apply(c, mat)


# --- per-frame ---------------------------------------------------------------

func _stretch(mi: MeshInstance3D, a: Vector3, b: Vector3) -> void:
	var d := b - a
	(mi.mesh as CylinderMesh).height = d.length()
	var y := d.normalized()
	var x := y.cross(Vector3.FORWARD).normalized()
	if x.length_squared() < 0.5:
		x = y.cross(Vector3.RIGHT).normalized()
	mi.global_transform = Transform3D(Basis(x, y, x.cross(y)), (a + b) * 0.5)


func _tension_color(mat: StandardMaterial3D, T: float, cap: float) -> void:
	var f: float = clampf(T / cap, 0.0, 1.0)
	mat.albedo_color = Color(0.85, 0.85, 0.82).lerp(Color(0.95, 0.15, 0.1), f)


func _pose(i: int) -> void:
	var r: Array = rows[i]
	var t: float = r[0]
	var pos := Vector3(r[4], r[5], 0.0)
	kite.position = pos
	kite.rotation = Vector3(0, 0, -deg_to_rad(r[3]))

	var pod_pos := Vector3(r[10], r[11], 0)
	pod.position = pod_pos
	_stretch(line_main, FAIRLEAD, Vector3(r[6], r[7], 0))
	_stretch(line_ctl_l, pod_pos, Vector3(r[8], r[9], -Y_CTL))
	_stretch(line_ctl_r, pod_pos, Vector3(r[8], r[9], Y_CTL))
	_tension_color(mat_main, r[12], 66.7e3)
	_tension_color(mat_ctl, r[13], 33.3e3)

	# camera: establishing wide from the deck, then an orbiting tracker
	# framing both the kite and the winchlet pod below it
	var focus := (pos + pod_pos) * 0.5
	var az := 0.35 + 0.06 * t
	var track := focus + 80.0 * Vector3(cos(az), 0.06, sin(az))
	var wide := Vector3(35.0, 14.0, 55.0)
	var w := clampf((t - 4.0) / 2.5, 0.0, 1.0)
	w = w * w * (3.0 - 2.0 * w)
	cam.position = wide.lerp(track, w)
	cam.look_at(focus, Vector3.UP)

	var gust := "  << GUST >>" if (t >= 15.0 and t <= 21.0) else ""
	hud.text = ("Mk V «Manta» — pod rig gust replay (l1_trim dynamics)\n"
		+ "t %5.1f s   wind %4.1f m/s%s\n" % [t, r[1], gust]
		+ "alpha %5.1f°   theta %5.1f°   alt %3.0f m\n" % [r[2], r[3], r[5]]
		+ "T_main %5.1f kN   T_ctl %4.1f kN   pod @ 50 m — winches LOCKED"
			% [r[12] / 1e3, r[13] / 1e3])
