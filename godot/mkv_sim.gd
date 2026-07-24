extends Node3D

# Mk V live sim — GDScript port of kytoon.solvers.l1_trim._derivs.
#
# The Python solver is the reference; this file must remain a line-for-line
# port of the 6-state longitudinal model (x, z, theta, u, w, omega), with
# parameters loaded from mkv_sim_params.json (export_sim_params.py). Verify
# after any model change:
#
#   godot --path godot mkv_sim.tscn --audio-driver Dummy -- \
#       --selftest=C:/path/out.csv        # locked-winch gust, mirrors
#                                         # renders/mkv_replay.csv
#
# Interactive (default, run from the editor or `godot --path godot mkv_sim.tscn`):
#   Up/Down     wind +/- 0.5 m/s
#   W/S         winchlet in/out (0.5 m/s, changes trim alpha)
#   G           fire a +6 m/s 1-cos gust (6 s)
#   R           reset to trim      Space: pause
#
# Demo capture: -- --demo-out=C:/path/frames  (scripted wind/winch sequence)

const DT := 1.0 / 240.0            # physics substep
const WINCH_RATE := 0.5            # m/s
const MK_V_COLOR := Color("4a3aa7")

var P: Dictionary                   # model parameters (JSON)
var s: Array[float] = []            # [x, z, theta, u, w, om]
var l0m: float
var l0c: float
var l0c_trim: float
var wind_base := 12.0
var sim_t := 0.0
var gust_t0 := -1e9
var paused := false
var mode := "interactive"           # | "selftest" | "demo"
var out_path := ""
var demo_frame := 0

var kite: Node3D
var pod: MeshInstance3D
var cam: Camera3D
var hud: Label
var line_main: MeshInstance3D
var line_ctl_l: MeshInstance3D
var line_ctl_r: MeshInstance3D
var mat_main: StandardMaterial3D
var mat_ctl: StandardMaterial3D
var last_alpha := 0.0
var last_tm := 0.0
var last_tc := 0.0


func _ready() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--selftest="):
			mode = "selftest"
			out_path = arg.split("=")[1]
		elif arg.begins_with("--demo-out="):
			mode = "demo"
			out_path = arg.split("=")[1]
	_load_params()
	_reset()
	if mode == "selftest":
		_run_selftest()
		return
	_build()
	if mode == "demo":
		DirAccess.make_dir_recursive_absolute(out_path)
		_run_demo()


func _load_params() -> void:
	var f := FileAccess.open("res://mkv_sim_params.json", FileAccess.READ)
	P = JSON.parse_string(f.get_as_text())


func _reset() -> void:
	s.assign(P["init"]["state"])
	l0m = P["init"]["l0_main"]
	l0c = P["init"]["l0_ctl"]
	l0c_trim = l0c
	wind_base = P["init"]["wind"]
	sim_t = 0.0
	gust_t0 = -1e9


func wind_now() -> float:
	var t := sim_t - gust_t0
	if t >= 0.0 and t <= 6.0:
		return wind_base + 3.0 * (1.0 - cos(TAU * t / 6.0))
	return wind_base


# --- physics: port of l1_trim._derivs --------------------------------------

func _interp(xs: Array, ys: Array, x: float) -> float:
	var n: int = xs.size()
	if x <= xs[0]:
		return ys[0]
	if x >= xs[n - 1]:
		return ys[n - 1]
	var i := 0
	while xs[i + 1] < x:
		i += 1
	var f: float = (x - xs[i]) / (xs[i + 1] - xs[i])
	return lerp(float(ys[i]), float(ys[i + 1]), f)


func _rot(p: Array, th: float) -> Vector2:
	var c := cos(th)
	var sn := sin(th)
	return Vector2(p[0] * c + p[1] * sn, -p[0] * sn + p[1] * c)


func _derivs(st: Array[float], U: float) -> Dictionary:
	var x := st[0]
	var z := st[1]
	var th := st[2]
	var u := st[3]
	var w := st[4]
	var om := st[5]
	var wa := Vector2(U - u, -w)
	var va: float = max(wa.length(), 1e-6)
	var alpha := rad_to_deg(th + atan2(wa.y, wa.x))
	var cl := _interp(P["alphas"], P["cl"], alpha)
	var cd := _interp(P["alphas"], P["cd"], alpha)
	var cm := _interp(P["alphas"], P["cm"], alpha)
	var S: float = P["S"]
	var c_ref: float = P["c_ref"]
	var q: float = 0.5 * P["rho"] * va * va
	var dhat := wa / va
	var lhat := Vector2(-dhat.y, dhat.x)
	var F := q * S * (cd * dhat + cl * lhat)
	var M: float = q * S * c_ref * (cm + P["cm_q"] * om * c_ref / (2.0 * va))
	var g: float = P["g"]
	for pair in [[P["r_skin"], -P["m_skin"] * g],
				 [P["p_main"], -P["m_pod"] * g],
				 [P["r_cb"], P["buoyancy_n"]]]:
		var rw := _rot(pair[0], th)
		F.y += pair[1]
		M += rw.y * 0.0 - rw.x * pair[1]
	# main line to the ship fairlead
	var anchor := Vector2(P["fairlead"][0], P["fairlead"][1])
	var rw_m := _rot(P["p_main"], th)
	var r_m := Vector2(x, z) + rw_m
	var v_m := Vector2(u, w) + om * Vector2(rw_m.y, -rw_m.x)
	var d := anchor - r_m
	var dist := d.length()
	var uv := d / dist
	var t_main := 0.0
	if dist > l0m:
		t_main = max(P["k_main"] * (dist - l0m)
			- P["c_line"] * v_m.dot(uv), 0.0)
		F += t_main * uv
		M += rw_m.y * (t_main * uv).x - rw_m.x * (t_main * uv).y
	# control pair from the pod riding the main line
	var pod_p := r_m + float(P["pod_standoff"]) * uv
	var rw_c := _rot(P["p_ctl"], th)
	var r_c := Vector2(x, z) + rw_c
	var v_c := Vector2(u, w) + om * Vector2(rw_c.y, -rw_c.x)
	var v := pod_p - r_c
	var v_len := v.length()
	var dist3: float = sqrt(v_len * v_len + P["y_ctl"] * P["y_ctl"])
	var t_ctl := 0.0
	if dist3 > l0c:
		var uvc := v / v_len
		t_ctl = max(P["k_ctl"] * (dist3 - l0c)
			- P["c_line"] * (v_c - v_m).dot(uvc), 0.0)
		var fc := t_ctl * (v_len / dist3) * uvc
		F += fc
		M += rw_c.y * fc.x - rw_c.x * fc.y
	var rcg := _rot(P["r_cg"], th)
	var m_cg: float = M - (rcg.y * F.x - rcg.x * F.y)
	return {
		"d": [u, w, om, F.x / (P["m_total"] + P["m_added_x"]),
			  F.y / (P["m_total"] + P["m_added_z"]),
			  m_cg / (P["i_yy"] + P["i_added"])],
		"alpha": alpha, "t_main": t_main, "t_ctl": t_ctl,
		"r_m": r_m, "r_c": r_c, "pod": pod_p,
	}


func _step(dt: float) -> Dictionary:
	var U := wind_now()
	var out := _derivs(s, U)
	var k1: Array = out["d"]
	var k2: Array = _derivs(_madd(s, k1, dt / 2.0), U)["d"]
	var k3: Array = _derivs(_madd(s, k2, dt / 2.0), U)["d"]
	var k4: Array = _derivs(_madd(s, k3, dt), U)["d"]
	for i in 6:
		s[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
	sim_t += dt
	return out


func _madd(a: Array[float], b: Array, f: float) -> Array[float]:
	var r: Array[float] = []
	for i in 6:
		r.append(a[i] + b[i] * f)
	return r


# --- headless self-test ------------------------------------------------------
# Mirrors export_mkv_replay.py: locked winches, gust at t=15..21, 45 s,
# recorded every 0.1 s. Compare the CSV against renders/mkv_replay.csv.

func _run_selftest() -> void:
	gust_t0 = 15.0
	var f := FileAccess.open(out_path, FileAccess.WRITE)
	f.store_line("t,U,alpha,theta_deg,x,z,T_main,T_ctl")
	var dt := 0.004
	var rec := 0.0
	while sim_t < 45.0:
		var out := _step(dt)
		if sim_t >= rec:
			f.store_line("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f"
				% [sim_t, wind_now(), out["alpha"], rad_to_deg(s[2]),
				   s[0], s[1], out["t_main"], out["t_ctl"]])
			rec += 0.1
	f.close()
	print("selftest written: " + out_path)
	get_tree().quit()


# --- interactive / demo ------------------------------------------------------

func _physics_process(delta: float) -> void:
	if mode != "interactive" or paused:
		return
	_handle_input(delta)
	var out: Dictionary
	var n := clampi(roundi(delta / DT), 1, 12)
	for i in n:
		out = _step(DT)
	_update_visuals(out)


func _handle_input(delta: float) -> void:
	if Input.is_key_pressed(KEY_UP):
		wind_base = clampf(wind_base + 2.0 * delta, 2.0, 26.0)
	if Input.is_key_pressed(KEY_DOWN):
		wind_base = clampf(wind_base - 2.0 * delta, 2.0, 26.0)
	if Input.is_key_pressed(KEY_W):
		l0c = clampf(l0c - WINCH_RATE * delta, l0c_trim - 3.0, l0c_trim + 3.0)
	if Input.is_key_pressed(KEY_S):
		l0c = clampf(l0c + WINCH_RATE * delta, l0c_trim - 3.0, l0c_trim + 3.0)


func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_G:
				gust_t0 = sim_t
			KEY_R:
				_reset()
			KEY_SPACE:
				paused = not paused


func _run_demo() -> void:
	# scripted: settle, gust, winch out (depower), winch in, wind up
	var script := [
		[3.0, "", 0.0], [1.0, "gust", 0.0],
		[10.0, "", 0.0], [3.0, "winch_out", 0.0],
		[6.0, "", 0.0], [3.0, "winch_in", 0.0],
		[6.0, "", 0.0],
	]
	var fps := 30.0
	for seg in script:
		var dur: float = seg[0]
		if seg[1] == "gust":
			gust_t0 = sim_t
		var t_seg := 0.0
		while t_seg < dur:
			if seg[1] == "winch_out":
				l0c = clampf(l0c + WINCH_RATE / fps,
					l0c_trim - 3.0, l0c_trim + 3.0)
			elif seg[1] == "winch_in":
				l0c = clampf(l0c - WINCH_RATE / fps,
					l0c_trim - 3.0, l0c_trim + 3.0)
			var out: Dictionary
			for i in int(1.0 / fps / DT):
				out = _step(DT)
			_update_visuals(out)
			await get_tree().process_frame
			await RenderingServer.frame_post_draw
			var img: Image = get_viewport().get_texture().get_image()
			img.save_png("%s/frame_%04d.png" % [out_path, demo_frame])
			demo_frame += 1
			t_seg += 1.0 / fps
	print("demo captured %d frames" % demo_frame)
	get_tree().quit()


# --- scene (visual family of mkv_replay.gd) ---------------------------------

func _build() -> void:
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

	kite = Node3D.new()
	add_child(kite)
	var inner := Node3D.new()
	inner.rotation_degrees = Vector3(-90.0, 0.0, 0.0)
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

	mat_main = StandardMaterial3D.new()
	mat_main.roughness = 0.9
	mat_ctl = StandardMaterial3D.new()
	mat_ctl.roughness = 0.9
	line_main = _make_line(0.20, mat_main)
	line_ctl_l = _make_line(0.10, mat_ctl)
	line_ctl_r = _make_line(0.10, mat_ctl)

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


func _update_visuals(out: Dictionary) -> void:
	last_alpha = out["alpha"]
	last_tm = out["t_main"]
	last_tc = out["t_ctl"]
	var pos := Vector3(s[0], s[1], 0.0)
	kite.position = pos
	kite.rotation = Vector3(0, 0, -s[2])
	var pod_v: Vector2 = out["pod"]
	var pod_pos := Vector3(pod_v.x, pod_v.y, 0)
	pod.position = pod_pos
	var rm: Vector2 = out["r_m"]
	var rc: Vector2 = out["r_c"]
	_stretch(line_main, Vector3(P["fairlead"][0], P["fairlead"][1], 0),
		Vector3(rm.x, rm.y, 0))
	_stretch(line_ctl_l, pod_pos, Vector3(rc.x, rc.y, -P["y_ctl"]))
	_stretch(line_ctl_r, pod_pos, Vector3(rc.x, rc.y, P["y_ctl"]))
	_tension_color(mat_main, last_tm, P["wll_n"])
	_tension_color(mat_ctl, last_tc, P["ctl_cap_n"])

	var focus := (pos + pod_pos) * 0.5
	var az := 0.35 + 0.05 * sim_t
	cam.position = focus + 80.0 * Vector3(cos(az), 0.06, sin(az))
	cam.look_at(focus, Vector3.UP)

	var gust := "  << GUST >>" if (sim_t - gust_t0) <= 6.0 else ""
	var winch := l0c - l0c_trim
	hud.text = ("Mk V «Manta» — LIVE sim (GDScript port of l1_trim)\n"
		+ "t %5.1f s   wind %4.1f m/s%s\n" % [sim_t, wind_now(), gust]
		+ "alpha %5.1f°   theta %5.1f°   alt %3.0f m\n"
			% [last_alpha, rad_to_deg(s[2]), s[1]]
		+ "T_main %5.1f kN   T_ctl %4.1f kN   winch %+0.2f m\n"
			% [last_tm / 1e3, last_tc / 1e3, winch]
		+ "Up/Dn wind   W/S winch   G gust   R reset   Space pause")
