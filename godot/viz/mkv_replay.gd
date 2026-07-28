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
	add_child(KytoonWorld.environment())
	KytoonWorld.lights(self)
	add_child(KytoonWorld.sea())
	add_child(KytoonWorld.ship())

	# kite — the wrapper carries the -90 deg X that takes the exporter's
	# z-up loft into glTF's Y-up
	kite = Node3D.new()
	add_child(kite)
	var wrapper := KytoonWorld.kite("mkv", KytoonWorld.MK_COLOR["V"])
	if wrapper != null:
		kite.add_child(wrapper)

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


func _make_line(radius: float, mat: StandardMaterial3D) -> MeshInstance3D:
	var mi := KytoonWorld.line_mesh(radius, mat)
	add_child(mi)
	return mi


# --- per-frame ---------------------------------------------------------------

func _pose(i: int) -> void:
	var r: Array = rows[i]
	var t: float = r[0]
	var pos := Vector3(r[4], r[5], 0.0)
	kite.position = pos
	kite.rotation = Vector3(0, 0, -deg_to_rad(r[3]))

	var pod_pos := Vector3(r[10], r[11], 0)
	pod.position = pod_pos
	KytoonWorld.stretch(line_main, FAIRLEAD, Vector3(r[6], r[7], 0))
	KytoonWorld.stretch(line_ctl_l, pod_pos, Vector3(r[8], r[9], -Y_CTL))
	KytoonWorld.stretch(line_ctl_r, pod_pos, Vector3(r[8], r[9], Y_CTL))
	KytoonWorld.tension_color(mat_main, r[12], 66.7e3)
	KytoonWorld.tension_color(mat_ctl, r[13], 33.3e3)

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
