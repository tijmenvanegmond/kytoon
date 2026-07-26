extends Node3D

# Mk V 6-DOF sim — GDScript port of kytoon.solvers.l1_sim3d.
#
# The Python solver is the reference. This is a PORT: same equations, same
# coefficients, same sign conventions, verified by --selftest3d against
# renders/mkv_sim3d_replay.csv. Do not add or "fix" physics here — change
# l1_sim3d, re-export, re-run the gate.
#
#   godot --path godot sim/mkv_sim3d.tscn --headless -- --selftest3d=out.csv
#
# State is a flat 13-array mirroring the Python layout so the two can be
# read side by side:
#   [0:3] position, world       [3:7] quaternion (w,x,y,z), body->world
#   [7:10] velocity, BODY       [10:13] angular velocity, BODY
#
# Body axes are the loft's own: x downstream, y starboard, z up. That is
# NOT Godot's frame; `_to_godot` maps it for drawing only, never for
# physics.
#
# Two things that keep this honest and short:
#   * M is constant in body axes, so the exporter ships M_INV and the
#     equation of motion is one 6x6 matvec — no linear solver in GDScript.
#   * Kirchhoff form: added mass (~12x structural in roll) appears in the
#     Coriolis terms too, built from the SAME M. Dropping that is what
#     makes a big light wing spin up wrongly under combined roll+yaw.

const N_STATE := 13
const DT := 1.0 / 240.0

var P: Dictionary
var s: Array[float] = []
var l0 := [0.0, 0.0, 0.0]          # rest lengths: PORT, MAIN, STBD
var l0_trim := [0.0, 0.0, 0.0]
var wind_speed := 12.0
var wind_azimuth := 0.0            # radians, 0 = along +x
var sim_t := 0.0

var mode := "interactive"
var out_path := ""

# cached params
var m_inv: Array = []
var m6: Array = []
var p_att: Array = []              # PORT, MAIN, STBD in body axes
var alphas: Array = []
var cl_t: Array = []
var cd_t: Array = []
var cm_t: Array = []

const PORT := 0
const MAIN := 1
const STBD := 2


func _ready() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--selftest3d="):
			mode = "selftest3d"
			out_path = arg.split("=")[1]
		elif arg.begins_with("--shots="):
			mode = "shots"
			out_path = arg.split("=")[1]
	_load_params()
	_reset()
	if mode == "selftest3d":
		_run_selftest3d()
		return
	_build_visuals()
	if mode == "shots":
		_run_shots()


func _load_params() -> void:
	var f := FileAccess.open("res://data/mkv_sim3d_params.json",
		FileAccess.READ)
	P = JSON.parse_string(f.get_as_text())
	m_inv = P["m6_inv"]
	m6 = P["m6"]
	p_att = [P["p_port"], P["p_main"], P["p_stbd"]]
	alphas = P["alphas"]
	cl_t = P["cl"]
	cd_t = P["cd"]
	cm_t = P["cm"]


func _reset() -> void:
	var init: Dictionary = P["init"]
	s.clear()
	for v in init["pos"]:
		s.append(float(v))
	for v in init["quat"]:
		s.append(float(v))
	for _i in 6:
		s.append(0.0)
	l0 = [float(init["l0"][0]), float(init["l0"][1]), float(init["l0"][2])]
	l0_trim = l0.duplicate()
	wind_speed = float(P["wind_ref"])
	wind_azimuth = 0.0
	sim_t = 0.0


# --- small helpers -----------------------------------------------------------

func _v3(a) -> Vector3:
	return Vector3(float(a[0]), float(a[1]), float(a[2]))


func _pos() -> Vector3:
	return Vector3(s[0], s[1], s[2])


func _quat() -> Quaternion:
	# JSON is (w, x, y, z); Godot's constructor is (x, y, z, w)
	return Quaternion(s[4], s[5], s[6], s[3]).normalized()


func _vel_body() -> Vector3:
	return Vector3(s[7], s[8], s[9])


func _omega_body() -> Vector3:
	return Vector3(s[10], s[11], s[12])


func wind_vector() -> Vector3:
	return Vector3(wind_speed * cos(wind_azimuth),
		wind_speed * sin(wind_azimuth), 0.0)


func _interp(xs: Array, ys: Array, x: float) -> float:
	var n: int = xs.size()
	if x <= float(xs[0]):
		return float(ys[0])
	if x >= float(xs[n - 1]):
		return float(ys[n - 1])
	var i := 0
	while float(xs[i + 1]) < x:
		i += 1
	var f: float = (x - float(xs[i])) / (float(xs[i + 1]) - float(xs[i]))
	return lerpf(float(ys[i]), float(ys[i + 1]), f)


func _matvec6(m: Array, v: Array) -> Array:
	var out: Array[float] = []
	for i in 6:
		var acc := 0.0
		var row: Array = m[i]
		for j in 6:
			acc += float(row[j]) * float(v[j])
		out.append(acc)
	return out


# --- forces: port of l1_rig3d.aero_wrench / line_wrench + l1_dyn3d ----------

func _aero_wrench(rot: Quaternion, v_rel_world: Vector3) -> Array:
	var v_body := rot.inverse() * v_rel_world
	var speed: float = maxf(v_body.length(), 1e-9)
	var alpha := atan2(v_body.z, v_body.x)
	var beta := asin(clampf(v_body.y / speed, -1.0, 1.0))
	var a_deg := rad_to_deg(alpha)
	var cl := _interp(alphas, cl_t, a_deg)
	var cd := _interp(alphas, cd_t, a_deg)
	var cm := _interp(alphas, cm_t, a_deg)
	var S: float = P["S"]
	var b_ref: float = P["b_ref"]
	var c_ref: float = P["c_ref"]
	var q: float = 0.5 * float(P["rho"]) * speed * speed

	var d_hat := v_body / speed
	var z_b := Vector3(0, 0, 1)
	var l_vec := z_b - z_b.dot(d_hat) * d_hat
	var l_hat := l_vec / maxf(l_vec.length(), 1e-9)
	var y_hat := -d_hat.cross(l_hat)          # +y at beta = 0

	# Sign conventions come from the exporter's derivatives: CY carries a
	# single beta flip, Cl/Cn carry two that cancel. Do not adjust here.
	var f_body := q * S * (cd * d_hat + cl * l_hat
		- float(P["cy_beta"]) * beta * y_hat)
	var m_body := Vector3(
		q * S * b_ref * float(P["cl_beta"]) * beta,
		q * S * c_ref * cm,
		q * S * b_ref * float(P["cn_beta"]) * beta)
	# aero acts at the body origin; shift to the CG
	m_body += (-_v3(P["r_cg"])).cross(f_body)
	return [rot * f_body, rot * m_body, a_deg, rad_to_deg(beta)]


func _rate_wrench(rot: Quaternion, speed: float) -> Vector3:
	if speed < 1e-6:
		return Vector3.ZERO
	var w := _omega_body()
	var S: float = P["S"]
	var b_ref: float = P["b_ref"]
	var c_ref: float = P["c_ref"]
	var qd: float = 0.5 * float(P["rho"]) * speed * speed
	var p_hat := w.x * b_ref / (2.0 * speed)
	var q_hat := w.y * c_ref / (2.0 * speed)
	var r_hat := w.z * b_ref / (2.0 * speed)
	var m_body := Vector3(
		qd * S * b_ref * (float(P["cl_p"]) * p_hat
			+ float(P["cl_r"]) * r_hat),
		qd * S * c_ref * (float(P["cm_q"]) * q_hat),
		qd * S * b_ref * (float(P["cn_p"]) * p_hat
			+ float(P["cn_r"]) * r_hat))
	return rot * m_body


## Three straight elastic lines: main to the fairlead, two control lines to
## the pod riding it. Returns [force, moment, tensions, pod].
func _line_wrench(pos: Vector3, rot: Quaternion) -> Array:
	var anchor := _v3(P["fairlead"])
	var cg := pos + rot * _v3(P["r_cg"])
	var ea_main: float = P["ea_main"]
	var ea_ctl: float = P["ea_ctl"]

	var p_main := pos + rot * _v3(p_att[MAIN])
	var to_anchor := anchor - p_main
	var dist: float = to_anchor.length()
	var u_main := to_anchor / maxf(dist, 1e-9)
	var k_main: float = ea_main / maxf(l0[MAIN], 1.0)
	var t_main: float = maxf(k_main * (dist - l0[MAIN]), 0.0)

	var standoff: float = minf(float(P["pod_standoff"]), 0.9 * dist)
	var pod := p_main + standoff * u_main

	var force := t_main * u_main
	var moment := (p_main - cg).cross(force)
	var tensions := [0.0, t_main, 0.0]

	for side in [PORT, STBD]:
		var att := pos + rot * _v3(p_att[side])
		var d := pod - att
		var length: float = d.length()
		var k_ctl: float = ea_ctl / maxf(l0[side], 1.0)
		var t: float = maxf(k_ctl * (length - l0[side]), 0.0)
		var f := t * d / maxf(length, 1e-9)
		force += f
		moment += (att - cg).cross(f)
		tensions[side] = t
	return [force, moment, tensions, pod]


func _line_damping(pos: Vector3, rot: Quaternion, vel_world: Vector3,
		omega_world: Vector3, pod: Vector3) -> Array:
	var anchor := _v3(P["fairlead"])
	var cg := pos + rot * _v3(P["r_cg"])
	var c_line: float = P["c_line"]
	var force := Vector3.ZERO
	var moment := Vector3.ZERO
	for pair in [[p_att[MAIN], anchor], [p_att[PORT], pod],
				 [p_att[STBD], pod]]:
		var att := pos + rot * _v3(pair[0])
		var d: Vector3 = pair[1] - att
		var n: float = d.length()
		if n < 1e-9:
			continue
		var u := d / n
		var v_att := vel_world + omega_world.cross(att - cg)
		# u points attach -> far end, so v.u > 0 means the line is
		# SHORTENING and tension must drop
		var f := -c_line * v_att.dot(u) * u
		force += f
		moment += (att - cg).cross(f)
	return [force, moment]


# --- equations of motion ------------------------------------------------------

func _derivatives(st: Array[float]) -> Array:
	var rot := Quaternion(st[4], st[5], st[6], st[3]).normalized()
	var pos := Vector3(st[0], st[1], st[2])
	var vel_b := Vector3(st[7], st[8], st[9])
	var omg_b := Vector3(st[10], st[11], st[12])
	var vel_w := rot * vel_b
	var omg_w := rot * omg_b

	var v_rel := wind_vector() - vel_w
	var aero := _aero_wrench(rot, v_rel)
	var lines := _line_wrench(pos, rot)
	var damp := _line_damping(pos, rot, vel_w, omg_w, lines[3])
	var m_rate := _rate_wrench(rot, v_rel.length())

	var g: float = P["g"]
	var f_world: Vector3 = aero[0] + lines[0] + damp[0] \
		+ Vector3(0, 0, -float(P["m_total"]) * g) \
		+ Vector3(0, 0, float(P["buoyancy_n"]))
	var m_world: Vector3 = aero[1] + lines[1] + damp[1] + m_rate \
		+ (rot * (_v3(P["r_cb"]) - _v3(P["r_cg"]))).cross(
			Vector3(0, 0, float(P["buoyancy_n"])))

	var f_b := rot.inverse() * f_world
	var m_b := rot.inverse() * m_world

	# Coriolis from the SAME 6x6: momenta first, then the cross terms.
	# C(nu)*nu = [omega x lin ; v x lin + omega x ang]
	var nu := [vel_b.x, vel_b.y, vel_b.z, omg_b.x, omg_b.y, omg_b.z]
	var mom := _matvec6(m6, nu)
	var lin := Vector3(mom[0], mom[1], mom[2])
	var ang := Vector3(mom[3], mom[4], mom[5])
	var c_lin := omg_b.cross(lin)
	var c_ang := vel_b.cross(lin) + omg_b.cross(ang)

	var tau := [f_b.x - c_lin.x, f_b.y - c_lin.y, f_b.z - c_lin.z,
				m_b.x - c_ang.x, m_b.y - c_ang.y, m_b.z - c_ang.z]
	var acc := _matvec6(m_inv, tau)

	# quaternion rate: 0.5 * q (x) (0, omega_body), plus a unit-norm pull
	var pure := Quaternion(omg_b.x, omg_b.y, omg_b.z, 0.0)
	var dq := rot * pure
	var norm_err: float = 1.0 - (st[3] * st[3] + st[4] * st[4]
		+ st[5] * st[5] + st[6] * st[6])

	var d: Array[float] = []
	d.resize(N_STATE)
	d[0] = vel_w.x
	d[1] = vel_w.y
	d[2] = vel_w.z
	d[3] = 0.5 * dq.w + norm_err * st[3]
	d[4] = 0.5 * dq.x + norm_err * st[4]
	d[5] = 0.5 * dq.y + norm_err * st[5]
	d[6] = 0.5 * dq.z + norm_err * st[6]
	for i in 6:
		d[7 + i] = float(acc[i])
	return [d, aero[2], aero[3], lines[2], lines[3]]


func _madd(a: Array[float], b: Array[float], f: float) -> Array[float]:
	var r: Array[float] = []
	r.resize(N_STATE)
	for i in N_STATE:
		r[i] = a[i] + b[i] * f
	return r


func _step(dt: float) -> Array:
	var o1 := _derivatives(s)
	var k1: Array[float] = o1[0]
	var k2: Array[float] = _derivatives(_madd(s, k1, dt / 2.0))[0]
	var k3: Array[float] = _derivatives(_madd(s, k2, dt / 2.0))[0]
	var k4: Array[float] = _derivatives(_madd(s, k3, dt))[0]
	for i in N_STATE:
		s[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
	var q := Quaternion(s[4], s[5], s[6], s[3]).normalized()
	s[3] = q.w
	s[4] = q.x
	s[5] = q.y
	s[6] = q.z
	sim_t += dt
	return o1


## (roll, pitch, yaw) in degrees for the 3-2-1 sequence l1_rig3d builds,
## read off the rotation matrix exactly as l1_sim3d._rpy_from_matrix does.
func rpy_deg() -> Vector3:
	var b := Basis(_quat())
	# Godot's Basis.x/.y/.z are COLUMNS, so numpy's r[i][j] (row i, col j)
	# is b[j][i]. Getting this backwards transposes the matrix, which
	# leaves yaw correct (it reads r00/r01, symmetric under the mistake)
	# while roll and pitch come out wrong by tens of degrees — exactly how
	# it first showed up against the Python reference.
	var r02 := b.z.x
	var r12 := b.z.y
	var r22 := b.z.z
	var r01 := b.y.x
	var r00 := b.x.x
	var theta := asin(clampf(r02, -1.0, 1.0))
	var phi := atan2(-r12, r22)
	var psi := atan2(-r01, r00)
	return Vector3(rad_to_deg(phi), rad_to_deg(theta), rad_to_deg(psi))


# --- headless parity gate -----------------------------------------------------
# Mirrors godot/tools/export_sim3d_replay.py: hold, then a rate-limited
# differential drum input. 0.10 m is deliberate — past ~0.15 m the
# paying-out control line goes slack and the kite departs (see
# KYTOON-PROJECT.md section 6), which would test the flat-plate
# extrapolation rather than the model.

func _run_selftest3d() -> void:
	const T_END := 40.0
	const T_STEP := 5.0
	const DIFF := 0.10
	const RAMP := 0.05
	var f := FileAccess.open(out_path, FileAccess.WRITE)
	f.store_line("t,roll,pitch,yaw,alpha,beta,x,y,z,T_port,T_main,T_stbd")
	var dt := 0.004
	var rec := 0.0
	while sim_t < T_END:
		if sim_t >= T_STEP:
			var d: float = minf(DIFF, RAMP * (sim_t - T_STEP))
			l0[PORT] = l0_trim[PORT] - d
			l0[STBD] = l0_trim[STBD] + d
		var out := _step(dt)
		if sim_t >= rec:
			var rpy := rpy_deg()
			var t: Array = out[3]
			f.store_line("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f"
				% [sim_t, rpy.x, rpy.y, rpy.z, out[1], out[2],
				   s[0], s[1], s[2], t[PORT], t[MAIN], t[STBD]])
			rec += 0.1
	f.close()
	print("selftest3d written: " + out_path)
	get_tree().quit()


# --- visuals ------------------------------------------------------------------
#
# Frame mapping. Body/sim axes are x downstream, y starboard, z up; Godot
# is X right, Y up, Z toward the viewer. T maps sim -> Godot as
# (x, y, z) -> (x, z, -y), which is right-handed. That is *exactly* what
# KytoonWorld.kite()'s -90 deg X wrapper already applies to the glb, so
# the outer node carries T * R_sim * T^-1 and the wrapper supplies the
# remaining T. Physics never touches this.

const T_SIM_TO_GODOT := Basis(Vector3(1, 0, 0), Vector3(0, 0, -1),
	Vector3(0, 1, 0))
const DRUM_RATE := 0.05            # m/s, the winchlet is rate limited
const DIFF_LIMIT := 0.25

var kite_node: Node3D
var pod_node: MeshInstance3D
var cam: Camera3D
var hud: Label
var line_nodes := []
var line_mats := []
var paused := false
var diff_cmd := 0.0
var common_cmd := 0.0
var last_out: Array = []


static func to_godot(v: Vector3) -> Vector3:
	return Vector3(v.x, v.z, -v.y)


func _build_visuals() -> void:
	add_child(KytoonWorld.environment())
	KytoonWorld.lights(self)
	add_child(KytoonWorld.sea())
	var ship := KytoonWorld.ship()
	ship.position = to_godot(_v3(P["fairlead"]))
	add_child(ship)

	var holder := Node3D.new()
	add_child(holder)
	var wrapper := KytoonWorld.kite("mkv", KytoonWorld.MK_COLOR["V"])
	if wrapper != null:
		holder.add_child(wrapper)
	kite_node = holder

	for i in 3:
		var mat := StandardMaterial3D.new()
		mat.roughness = 0.9
		var mi := KytoonWorld.line_mesh(0.20 if i == MAIN else 0.10, mat)
		add_child(mi)
		line_nodes.append(mi)
		line_mats.append(mat)

	pod_node = MeshInstance3D.new()
	var pm := CapsuleMesh.new()
	pm.radius = 0.45
	pm.height = 1.8
	pod_node.mesh = pm
	var pmat := StandardMaterial3D.new()
	pmat.albedo_color = Color(0.9, 0.55, 0.1)
	pod_node.material_override = pmat
	add_child(pod_node)

	cam = Camera3D.new()
	cam.fov = 50.0
	cam.far = 5000.0
	add_child(cam)

	var canvas := CanvasLayer.new()
	add_child(canvas)
	hud = Label.new()
	hud.position = Vector2(24, 18)
	hud.add_theme_font_size_override("font_size", 20)
	hud.add_theme_color_override("font_color", Color(1, 1, 1, 0.95))
	hud.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.9))
	hud.add_theme_constant_override("outline_size", 8)
	canvas.add_child(hud)


## Scripted capture: settle at trim, then steer to the reachable limit so
## the frame mapping and the roll direction can be eyeballed against the
## numbers the parity gate already agrees on.
func _run_shots() -> void:
	DirAccess.make_dir_recursive_absolute(out_path)
	var fps := 30.0
	var frame := 0
	for phase in [[3.0, 0.0], [12.0, 0.10]]:
		var target: float = phase[1]
		var t_seg := 0.0
		while t_seg < float(phase[0]):
			diff_cmd = move_toward(diff_cmd, target, DRUM_RATE / fps)
			l0[PORT] = l0_trim[PORT] - diff_cmd
			l0[STBD] = l0_trim[STBD] + diff_cmd
			for _i in int(1.0 / fps / DT):
				last_out = _step(DT)
			_update_visuals()
			await get_tree().process_frame
			await RenderingServer.frame_post_draw
			get_viewport().get_texture().get_image().save_png(
				"%s/frame_%03d.png" % [out_path, frame])
			frame += 1
			t_seg += 1.0 / fps
	print("shots: %d frames, final roll %.2f deg" % [frame, rpy_deg().x])
	get_tree().quit()


func _physics_process(delta: float) -> void:
	if mode != "interactive" or paused:
		return
	_handle_input(delta)
	var n := clampi(roundi(delta / DT), 1, 12)
	for _i in n:
		last_out = _step(DT)
	_update_visuals()


func _handle_input(delta: float) -> void:
	if Input.is_key_pressed(KEY_UP):
		wind_speed = clampf(wind_speed + 2.0 * delta, 2.0, 26.0)
	if Input.is_key_pressed(KEY_DOWN):
		wind_speed = clampf(wind_speed - 2.0 * delta, 2.0, 26.0)
	if Input.is_key_pressed(KEY_LEFT):
		wind_azimuth += deg_to_rad(15.0) * delta
	if Input.is_key_pressed(KEY_RIGHT):
		wind_azimuth -= deg_to_rad(15.0) * delta
	# steering: differential drum. Past ~0.1 m the paying-out line goes
	# slack and the rig departs — the HUD says so rather than the sim
	# preventing it.
	if Input.is_key_pressed(KEY_A):
		diff_cmd = clampf(diff_cmd + DRUM_RATE * delta, -DIFF_LIMIT,
			DIFF_LIMIT)
	if Input.is_key_pressed(KEY_D):
		diff_cmd = clampf(diff_cmd - DRUM_RATE * delta, -DIFF_LIMIT,
			DIFF_LIMIT)
	if Input.is_key_pressed(KEY_W):
		common_cmd = clampf(common_cmd - DRUM_RATE * delta, -1.0, 1.0)
	if Input.is_key_pressed(KEY_S):
		common_cmd = clampf(common_cmd + DRUM_RATE * delta, -1.0, 1.0)
	l0[PORT] = l0_trim[PORT] - diff_cmd + common_cmd
	l0[STBD] = l0_trim[STBD] + diff_cmd + common_cmd


func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_R:
				_reset()
				diff_cmd = 0.0
				common_cmd = 0.0
			KEY_SPACE:
				paused = not paused
			KEY_ESCAPE:
				get_tree().quit()


func _update_visuals() -> void:
	if last_out.is_empty():
		return
	var rot := _quat()
	var pos := _pos()
	var g_pos := to_godot(pos)
	kite_node.position = g_pos
	kite_node.basis = T_SIM_TO_GODOT * Basis(rot) \
		* T_SIM_TO_GODOT.inverse()

	var tens: Array = last_out[3]
	var pod: Vector3 = last_out[4]
	pod_node.position = to_godot(pod)
	var anchor := _v3(P["fairlead"])
	var ends := [
		[pod, pos + rot * _v3(p_att[PORT])],
		[anchor, pos + rot * _v3(p_att[MAIN])],
		[pod, pos + rot * _v3(p_att[STBD])],
	]
	var caps := [float(P["ctl_cap_n"]) / 2.0, float(P["wll_n"]),
				 float(P["ctl_cap_n"]) / 2.0]
	for i in 3:
		KytoonWorld.stretch(line_nodes[i], to_godot(ends[i][0]),
			to_godot(ends[i][1]))
		KytoonWorld.tension_color(line_mats[i], float(tens[i]), caps[i])

	var focus := (g_pos + to_godot(pod)) * 0.5
	var az := 0.6 + 0.04 * sim_t
	cam.position = focus + 90.0 * Vector3(cos(az), 0.12, sin(az))
	cam.position.y = maxf(cam.position.y, 4.0)
	cam.look_at(focus, Vector3.UP)

	var rpy := rpy_deg()
	var slack := ""
	if minf(float(tens[PORT]), float(tens[STBD])) < 50.0:
		slack = "   ** CTL LINE SLACK — rig is departing **"
	hud.text = ("Mk V «Manta» — 6-DOF (GDScript port of l1_sim3d)\n"
		+ "t %5.1f s   wind %4.1f m/s @ %+5.1f°\n"
			% [sim_t, wind_speed, rad_to_deg(wind_azimuth)]
		+ "roll %+6.1f   pitch %+6.1f   yaw %+6.1f\n"
			% [rpy.x, rpy.y, rpy.z]
		+ "alpha %+6.1f   beta %+6.1f   alt %3.0f m\n"
			% [float(last_out[1]), float(last_out[2]), pos.z]
		+ "T  port %5.2f   main %5.1f   stbd %5.2f kN%s\n"
			% [float(tens[PORT]) / 1e3, float(tens[MAIN]) / 1e3,
			   float(tens[STBD]) / 1e3, slack]
		+ "drums: differential %+0.3f m   common %+0.3f m\n"
			% [diff_cmd, common_cmd]
		+ "A/D steer   W/S trim   Up/Dn wind   L/R wind dir   "
		+ "R reset   Space pause")
