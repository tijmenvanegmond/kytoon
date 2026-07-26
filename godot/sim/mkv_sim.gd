extends Node3D

## Mk V «Manta» — live longitudinal sim.
##
## The PHYSICS section below is a port of kytoon.solvers.l1_trim._derivs
## (6-state model: x, z, theta, u, w, omega) plus the lumped-mass
## segmented tether. The Python solver is the reference — do not add aero
## or buoyancy physics here that l1_trim doesn't have, and re-run the
## parity check after touching anything under "PHYSICS":
##
##   godot --path godot sim/mkv_sim.tscn --headless -- --selftest=out.csv
##   # then diff against renders/mkv_replay.csv (see godot/README.md)
##
## Model parameters come from data/mkv_sim_params.json — regenerate with
## tools/export_sim_params.py after any spec or solver change.
##
## Headless modes:
##   --selftest=FILE        locked-winch gust, straight-line model (parity)
##   --recovery-test=FILE   full 400 -> 20 m winch-in, segmented line
##   --demo-out=DIR         scripted capture, one PNG per frame

const DT := 1.0 / 240.0            # physics substep
const WINCH_RATE := 0.5            # m/s, control winchlet
const MAIN_WINCH_RATE := 2.0       # m/s, main recovery winch
const T_TEND := 3000.0             # N, ctl drum constant-tension when docked
const ALPHA_HOLD_GAIN := 0.08      # m of ctl-line per (deg-error * s)
const FP_CL := 1.0                 # flat-plate lift peak, CL = FP_CL·sin2α
const FP_CD_MAX := 1.9             # bluff-body drag at 90°
const FP_BLEND := 12.0             # degrees of blend out of the table
const MAX_STEPS_PER_FRAME := 360   # frame-time guard for fast-forward
const TIME_RATES := [0.25, 0.5, 1.0, 2.0, 4.0, 8.0]

## Starting conditions worth flying. Line length is what the winch has
## paid out; the kite is placed along the trim elevation so it starts taut.
const SCENARIOS := [
	{"name": "cruise", "wind": 12.0, "line": 200.0},
	{"name": "loiter", "wind": 8.0, "line": 400.0},
	{"name": "capture hover", "wind": 5.0, "line": 25.0},
]

# segmented main line (lumped-mass): sag, weight, drag, honest slack.
# The selftest uses the straight-spring model for parity with l1_trim;
# interactive and recovery use the segments.
const SEG_TARGET_LEN := 35.0 / 3.0
const SEG_MAX := 36

var P: Dictionary                   # model parameters (JSON)
var s: Array[float] = []            # [x, z, theta, u, w, om]
var l0m: float
var l0m_full: float
var l0c: float
var l0c_trim: float
var ea_main: float                  # line law: k = EA / deployed length
var ea_ctl: float
var l_ctl0: float                   # nominal ctl 3D length at full standoff
var ctl_rest2: float                # l_ctl0^2 - standoff^2 (span+chord part)
var docked := false

var seg_mode := true
var nodes_p: Array[Vector2] = []    # [0]=fairlead ... [n_seg]=kite attach
var nodes_v: Array[Vector2] = []
var n_seg := 0

var payload_kg := 60.0
var payload_on_pod := false
var start_payload := -1.0           # >= 0 when set from the command line
var start_wind := -1.0
var wind_base := 12.0
var sim_t := 0.0
var gust_t0 := -1e9
var paused := false
var splashed := false
var trim_hold := false
var alpha_hold_deg := 6.0
var scenario := 0
var rate_idx := 2
var mode := "interactive"           # | "selftest" | "recovery" | "demo"
var out_path := ""
var demo_frame := 0

var kite: Node3D
var pod: MeshInstance3D
var cam: SimCamera
var hud: SimHud
var line_main: MeshInstance3D
var seg_lines: Array[MeshInstance3D] = []
var line_ctl_l: MeshInstance3D
var line_ctl_r: MeshInstance3D
var mat_main: StandardMaterial3D
var mat_ctl: StandardMaterial3D
var manta_type_loader: Node
var current_manta_type: String = "B"
var last_out: Dictionary = {}


func _ready() -> void:
	# Initialize Manta Type Loader
	var type_loader_scene = preload("res://common/manta_type_loader.tscn")
	manta_type_loader = type_loader_scene.instantiate()
	add_child(manta_type_loader)
	manta_type_loader.connect("manta_type_changed", Callable(self, "_on_manta_type_changed"))
	
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--selftest="):
			mode = "selftest"
			out_path = arg.split("=")[1]
		elif arg.begins_with("--recovery-test="):
			mode = "recovery"
			out_path = arg.split("=")[1]
		elif arg.begins_with("--demo-out="):
			mode = "demo"
			out_path = arg.split("=")[1]
		elif arg.begins_with("--fly-test="):
			mode = "fly"
			out_path = arg.split("=")[1]
		elif arg.begins_with("--wind="):
			start_wind = float(arg.split("=")[1])
		elif arg.begins_with("--payload="):
			start_payload = float(arg.split("=")[1])
		elif arg == "--payload-on-pod":
			payload_on_pod = true
		elif arg.begins_with("--manta-type="):
			var type_name = arg.split("=")[1]
			manta_type_loader.switch_to_type(type_name)
			current_manta_type = type_name
	_load_params()
	if start_payload >= 0.0:
		_apply_payload(start_payload)
	if mode == "selftest":
		# the gate always runs the spec config, whatever the flags say
		payload_on_pod = false
		_reset_reference()
		seg_mode = false     # parity mode: straight line, matches l1_trim
		_run_selftest()
		return
	if mode == "recovery":
		var keep := payload_kg
		_reset_reference()
		if start_payload >= 0.0:
			_apply_payload(keep)
		_run_recovery_test()
		return
	if mode == "fly":
		var hold := payload_kg
		_reset_reference()
		if start_payload >= 0.0:
			_apply_payload(hold)
		if start_wind > 0.0:
			wind_base = start_wind
		_run_fly_test()
		return
	_reset()
	_build()
	if mode == "demo":
		_ensure_out_dir(out_path)
		_run_demo()


func _load_params(type_name: String = "B") -> void:
	# Try to load Manta type-specific params first
	var manta_specs_file = FileAccess.open("res://data/manta_specs.json", FileAccess.READ)
	if manta_specs_file:
		var json = JSON.new()
		if json.parse(manta_specs_file.get_as_text()) == OK:
			var data = json.get_data()
			var specs = data.get("manta_specs", {})
			if specs.has(type_name):
				var spec = specs[type_name]
				# Build P dictionary from spec
				P = _build_params_from_spec(spec, type_name)
				current_manta_type = type_name
				print("Loaded Manta Type ", type_name, " parameters")
				return
	
	# Fallback to original Mk V params
	var f := FileAccess.open("res://data/mkv_sim_params.json", FileAccess.READ)
	if f == null:
		push_error("missing data/mkv_sim_params.json — run "
			+ "`python godot/tools/export_sim_params.py` from the repo root")
		get_tree().quit(1)
		return
	P = JSON.parse_string(f.get_as_text())
	ea_main = P["k_main"] * P["tether_length"]
	var dx: float = P["p_ctl"][0] - P["p_main"][0]
	var dz: float = P["p_ctl"][1] - P["p_main"][1]
	ctl_rest2 = P["y_ctl"] * P["y_ctl"] + dx * dx + dz * dz
	l_ctl0 = sqrt(P["pod_standoff"] * P["pod_standoff"] + ctl_rest2)
	ea_ctl = P["k_ctl"] * l_ctl0
	_apply_payload(P["payload_ref_kg"])


func _build_params_from_spec(spec: Dictionary, type_name: String) -> Dictionary:
	# Build simulation parameters from Manta spec JSON
	var P := {}
	
	# Basic properties
	P["name"] = spec.get("name", "Manta Type " + type_name)
	P["wing_area"] = spec.get("wing_area", 288.0)
	P["helium_volume"] = spec.get("helium_volume", 530.0)
	P["total_mass"] = spec.get("total_mass", 271.0)
	P["structure_mass"] = spec.get("structure_mass", 211.0)
	P["payload_ref_kg"] = spec.get("payload_mass", 60.0)
	P["rigging_kg"] = spec.get("rigging_mass", 50.0)
	
	# Fat wing properties
	if spec.has("fat_wing"):
		var fw = spec["fat_wing"]
		P["span"] = fw.get("span", 32.0)
		P["chord"] = fw.get("chord", 13.3)
		P["taper"] = fw.get("taper", 0.35)
		P["thickness_ratio"] = fw.get("thickness_ratio", 0.28)
		P["n_cells"] = fw.get("n_cells", 5)
		P["pressure_bar"] = fw.get("pressure_bar", 0.10)
		P["dihedral_deg"] = fw.get("dihedral_deg", 10.0)
	
	# Canopy properties
	if spec.has("canopy"):
		var c = spec["canopy"]
		P["cl_op"] = c.get("cl_op", 0.7)
		P["cl_max"] = c.get("cl_max", 1.1)
		P["cd_op"] = c.get("cd_op", 0.10)
		P["twin_skin"] = c.get("twin_skin", true)
	
	# Tether properties
	if spec.has("tether"):
		var t = spec["tether"]
		P["tether_length"] = t.get("length", 400.0)
		P["tether_diameter_mm"] = t.get("diameter_mm", 16.0)
		P["tether_linear_density"] = t.get("linear_density", 0.14)
		P["tether_mbl_kn"] = t.get("mbl_kn", 200.0)
		P["tether_safety_factor"] = t.get("safety_factor", 3.0)
		P["elevation_deg"] = t.get("elevation_deg", 50.0)
	
	# Bridle properties
	if spec.has("bridle"):
		var b = spec["bridle"]
		P["bridle_positions"] = b.get("positions", [0.12, 0.50, 0.88])
		P["bridle_chord_fraction"] = b.get("chord_fraction", 0.35)
		P["control_mbl_kn"] = b.get("control_mbl_kn", 50.0)
		P["pod_standoff"] = b.get("pod_standoff_m", 50.0)
	
	# Fin properties
	if spec.has("fin"):
		var f = spec["fin"]
		P["fin_area"] = f.get("area", 12.0)
		P["fin_arm"] = f.get("arm", 18.0)
	
	# Compute derived parameters (simplified - would need full l1_trim port)
	P["k_main"] = 100000.0  # Placeholder - should be computed from tether
	P["k_ctl"] = 50000.0   # Placeholder - should be computed from control lines
	
	# Mass properties
	P["m_skin"] = P["structure_mass"]
	P["m_pod"] = P["rigging_kg"] + P["payload_ref_kg"]
	
	# Geometry
	P["p_main"] = [0.0, 0.0]  # Main attachment point
	P["p_ctl"] = [1.0, 0.0]   # Control line attachment
	P["y_ctl"] = 5.0          # Control line span
	P["fairlead"] = [0.0, -50.0]  # Fairlead position
	P["r_skin"] = [0.0, 0.0]    # CG of skin
	
	# Aerodynamic limits
	P["wll_n"] = P["tether_mbl_kn"] * 1000.0 * P["tether_safety_factor"]
	P["ctl_cap_n"] = P["control_mbl_kn"] * 1000.0
	
	# Initial state (from L1 trim)
	P["init"] = {
		"state": [0.0, 0.0, 0.0, 12.0, 0.0, 0.0],  # x, z, theta, u, w, omega
		"l0_main": P["tether_length"] * 0.95,
		"l0_ctl": 50.0,
		"wind": 12.0
	}
	
	return P


func _on_manta_type_changed(type_name: String, config: Dictionary) -> void:
	current_manta_type = type_name
	_load_params(type_name)
	
	# Update kite color based on type
	if kite and kite.get_child_count() > 0:
		var kite_model = kite.get_child(0)
		if kite_model is MeshInstance3D:
			var color = KytoonWorld.MK_COLOR.get(type_name, KytoonWorld.MK_COLOR["B"])
			if kite_model.material_override:
				kite_model.material_override.albedo_color = color
			else:
				var mat = StandardMaterial3D.new()
				mat.albedo_color = color
				kite_model.material_override = mat
	
	# Update HUD to show current type
	if hud:
		hud.update_type_display(type_name, config.get("display_name", type_name))
	
	# Rebuild the sim with new parameters
	if mode == "interactive":
		_reset()
		_build()


## Payload is a sim OPTION, not a spec change: fly the design at other
## masses without editing specs/mk5_manta.yaml. Everything downstream of
## payload is recomputed exactly the way l1_trim.mass_props does it (the
## exporter asserts the rearrangement), so the flight model stays honest
## at any mass — the airframe, its buoyancy and its aero do not change.
func _apply_payload(kg: float) -> void:
	payload_kg = clampf(kg, 0.0, 1500.0)
	var m_skin: float = P["m_skin"]
	# on the pod the payload hangs on the tether, not on the wing: it
	# leaves the kite's mass, CG and inertia entirely and becomes a point
	# mass on the line (see _substep_segmented). Segmented line only —
	# the straight-line parity model has no line to hang it from.
	var carried: float = 0.0 if payload_on_pod else payload_kg
	var m_pod: float = carried + float(P["rigging_kg"])
	var m_total: float = m_skin + m_pod
	var p_main := Vector2(P["p_main"][0], P["p_main"][1])
	var r_skin := Vector2(P["r_skin"][0], P["r_skin"][1])
	var r_cg := (r_skin * m_skin + p_main * m_pod) / m_total
	P["m_pod"] = m_pod
	P["m_total"] = m_total
	P["r_cg"] = [r_cg.x, r_cg.y]
	P["i_yy"] = float(P["i_skin_own"]) \
		+ m_skin * r_skin.distance_squared_to(r_cg) \
		+ m_pod * p_main.distance_squared_to(r_cg)
	# ia_c, not m_added_z: the latter carries the cos²Γ fold projection
	# and i_added does not (see export_sim_params.py)
	P["i_added"] = float(P["ia_a"]) - 2.0 * float(P["ia_b"]) * r_cg.x \
		+ float(P["ia_c"]) * r_cg.x * r_cg.x + float(P["ia_d"])


## Net static lift of the whole system [kg] — payload counts wherever it
## hangs. Positive floats, negative must be flown.
func net_lift_kg() -> float:
	var hung: float = payload_kg if payload_on_pod else 0.0
	return float(P["buoyancy_n"]) / float(P["g"]) - float(P["m_total"]) - hung


## The exact l1_trim reconstruction: 12 m/s mission trim on the full
## tether. The parity gate and the recovery run START HERE — scenario
## presets are an interactive convenience and must never leak into them,
## or --selftest silently stops testing what it claims to.
func _reset_reference() -> void:
	_apply_payload(P["payload_ref_kg"])   # gates run the spec's payload
	s.assign(P["init"]["state"])
	l0m_full = P["tether_length"]
	l0m = P["init"]["l0_main"]
	l0c = P["init"]["l0_ctl"]
	l0c_trim = l0c
	wind_base = P["init"]["wind"]
	sim_t = 0.0
	gust_t0 = -1e9
	docked = false
	splashed = false
	paused = false
	_init_line()


## Interactive start: reference trim re-scaled onto the scenario's line
## length and wind, so the line begins taut whatever the length.
func _reset() -> void:
	var keep_payload := payload_kg      # a chosen payload survives R
	_reset_reference()
	_apply_payload(keep_payload)
	var sc: Dictionary = SCENARIOS[scenario]
	l0m = clampf(float(sc["line"]), 8.0, l0m_full)
	var anchor := Vector2(P["fairlead"][0], P["fairlead"][1])
	var r0 := Vector2(s[0], s[1]) - anchor
	var r_new := anchor + r0 * (l0m / maxf(r0.length(), 1e-6))
	s[0] = r_new.x
	s[1] = r_new.y
	wind_base = float(sc["wind"])
	_init_line()
	if hud:
		hud.reset_plot()


func time_scale() -> float:
	return TIME_RATES[rate_idx]


# ===========================================================================
# PHYSICS — port of kytoon.solvers.l1_trim. Keep in sync with the solver;
# the --selftest mode is the gate that proves it.
# ===========================================================================

func _kite_attach() -> Vector2:
	return Vector2(s[0], s[1]) + _rot(P["p_main"], s[2])


func _init_line() -> void:
	n_seg = clampi(int(ceil(l0m / SEG_TARGET_LEN)), 2, SEG_MAX)
	var a := Vector2(P["fairlead"][0], P["fairlead"][1])
	var b := _kite_attach()
	nodes_p.clear()
	nodes_v.clear()
	for i in n_seg + 1:
		nodes_p.append(a.lerp(b, float(i) / n_seg))
		nodes_v.append(Vector2.ZERO)


func _remesh_line() -> void:
	var n_new: int = clampi(int(ceil(l0m / SEG_TARGET_LEN)), 2, SEG_MAX)
	if n_new == n_seg:
		return
	# resample the old polyline by arc length
	var lens: Array[float] = [0.0]
	for i in n_seg:
		lens.append(lens[i] + (nodes_p[i + 1] - nodes_p[i]).length())
	var total: float = lens[n_seg]
	var new_p: Array[Vector2] = []
	var new_v: Array[Vector2] = []
	for j in n_new + 1:
		var target := total * float(j) / n_new
		var i := 0
		while i < n_seg - 1 and lens[i + 1] < target:
			i += 1
		var f := 0.0
		if lens[i + 1] > lens[i]:
			f = (target - lens[i]) / (lens[i + 1] - lens[i])
		new_p.append(nodes_p[i].lerp(nodes_p[i + 1], f))
		new_v.append(nodes_v[i].lerp(nodes_v[i + 1], f))
	nodes_p = new_p
	nodes_v = new_v
	n_seg = n_new


func wind_now() -> float:
	var t := sim_t - gust_t0
	if t >= 0.0 and t <= 6.0:
		return wind_base + 3.0 * (1.0 - cos(TAU * t / 6.0))
	return wind_base


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


## (CL, CD, Cm) at alpha [deg].
##
## INSIDE the exported table (l1_trim's validated band, −8…24°) this is
## the solver's own data interpolated — untouched, and the band the
## parity gate covers.
##
## OUTSIDE it, blend to a flat plate. l1_trim makes no claim out here and
## the table simply clamped, which let a stalled wing keep gliding on
## CD ≈ 0.05 at α = −60° and turned every slack-line upset into a clean,
## unrecoverable dive. A real fat wing at that attitude is a bluff body
## (CD ≈ 1.9) that decelerates and tumbles until buoyancy and the line
## take over. Cm still holds at the edge value — least trustworthy term,
## and drag is what dominates the recovery. SIM ONLY: these coefficients
## are not a design claim, and the HUD flags when you are out here.
func _aero(a: float) -> Vector3:
	var xs: Array = P["alphas"]
	var cl := _interp(xs, P["cl"], a)
	var cd := _interp(xs, P["cd"], a)
	var cm := _interp(xs, P["cm"], a)
	var lo: float = xs[0]
	var hi: float = xs[xs.size() - 1]
	if a >= lo and a <= hi:
		return Vector3(cl, cd, cm)
	var beyond: float = (a - hi) if a > hi else (lo - a)
	var w: float = clampf(beyond / FP_BLEND, 0.0, 1.0)
	var r := deg_to_rad(a)
	var cl_fp: float = FP_CL * sin(2.0 * r)
	var cd_fp: float = 0.06 + (FP_CD_MAX - 0.06) * sin(r) * sin(r)
	return Vector3(lerpf(cl, cl_fp, w), lerpf(cd, cd_fp, w), cm)


func _aero_extrapolated(a: float) -> bool:
	var xs: Array = P["alphas"]
	return a < float(xs[0]) or a > float(xs[xs.size() - 1])


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
	var co := _aero(alpha)
	var cl := co.x
	var cd := co.y
	var cm := co.z
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
	var k_main: float = ea_main / max(l0m, 5.0)      # line law k = EA/L
	if dist > l0m:
		t_main = max(k_main * (dist - l0m)
			- P["c_line"] * v_m.dot(uv), 0.0)
		F += t_main * uv
		M += rw_m.y * (t_main * uv).x - rw_m.x * (t_main * uv).y
	# control pair from the pod riding the main line; on recovery the pod
	# reaches the fairlead and DOCKS: it pins 3 m up-line of the fairlead
	# (ship-anchored — a kite-relative pod with a slack main invents
	# momentum) and the drum auto-tends
	var standoff: float = clampf(dist - 3.0, 0.5, float(P["pod_standoff"]))
	var is_docked := standoff < float(P["pod_standoff"]) - 0.01
	var k_ctl: float = ea_ctl / sqrt(standoff * standoff + ctl_rest2)
	var pod_p := r_m + standoff * uv
	var rw_c := _rot(P["p_ctl"], th)
	var r_c := Vector2(x, z) + rw_c
	var v_c := Vector2(u, w) + om * Vector2(rw_c.y, -rw_c.x)
	var v := pod_p - r_c
	var v_len := v.length()
	var dist3: float = sqrt(v_len * v_len + P["y_ctl"] * P["y_ctl"])
	var l0c_eff := l0c
	if is_docked:
		# constant-tension tend: a docked drum hauls slack IN as well as
		# paying out. (It used to be max(l0c, ...), which only tensioned
		# if the drum happened to be short — so a payed-out drum left the
		# control pair dead slack through the whole hover.)
		l0c_eff = dist3 - T_TEND / k_ctl
	var t_ctl := 0.0
	if dist3 > l0c_eff:
		var uvc := v / v_len
		var v_ref := Vector2.ZERO if is_docked else v_m
		t_ctl = max(k_ctl * (dist3 - l0c_eff)
			- P["c_line"] * (v_c - v_ref).dot(uvc), 0.0)
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
		"docked": is_docked, "dist": dist,
	}


func _step(dt: float) -> Dictionary:
	if seg_mode:
		return _step_segmented(dt)
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


# Segmented-line step: kite (same aero/buoyancy laws as _derivs — keep in
# sync) + lumped-mass line nodes, semi-implicit Euler on two substeps.
# The line has weight, drag, and honest slack; the pod rides the actual
# line shape and its control reaction pushes back on the line nodes.
func _step_segmented(dt: float) -> Dictionary:
	var out: Dictionary
	var seg_rest: float = l0m / maxi(n_seg, 2)
	var n_sub := 2                       # short segments = stiff springs
	if seg_rest < 22.0:
		n_sub = 4
	if seg_rest < 12.0:
		n_sub = 6
	for sub in n_sub:
		out = _substep_segmented(dt / n_sub)
	sim_t += dt
	return out


func _substep_segmented(dt: float) -> Dictionary:
	var U := wind_now()
	_remesh_line()
	var anchor := Vector2(P["fairlead"][0], P["fairlead"][1])
	nodes_p[0] = anchor
	nodes_v[0] = Vector2.ZERO
	var attach := _kite_attach()
	var rw_m := _rot(P["p_main"], s[2])
	var v_attach := Vector2(s[3], s[4]) + s[5] * Vector2(rw_m.y, -rw_m.x)
	nodes_p[n_seg] = attach
	nodes_v[n_seg] = v_attach

	var seg_rest: float = l0m / n_seg
	var k_seg: float = ea_main / seg_rest
	var m_node: float = P["tether_linear_density"] * seg_rest
	var c_seg: float = 2.0 * 0.05 * sqrt(k_seg * m_node)   # 5% structural

	# segment tensions (no compression)
	var seg_T: Array[float] = []
	var seg_dir: Array[Vector2] = []
	for i in n_seg:
		var d := nodes_p[i + 1] - nodes_p[i]
		var L := d.length()
		var u := d / maxf(L, 1e-9)
		var vrel := (nodes_v[i + 1] - nodes_v[i]).dot(u)
		var T := 0.0
		if L > seg_rest:
			T = maxf(k_seg * (L - seg_rest) + c_seg * vrel, 0.0)
		seg_T.append(T)
		seg_dir.append(u)

	# node forces: segment pull + weight + cylinder drag
	var node_F: Array[Vector2] = []
	node_F.resize(n_seg + 1)
	for i in n_seg + 1:
		node_F[i] = Vector2.ZERO
	for i in n_seg:
		node_F[i] += seg_T[i] * seg_dir[i]
		node_F[i + 1] -= seg_T[i] * seg_dir[i]
	var cd_cyl: float = 0.5 * P["rho"] * 1.1 * P["tether_diameter_m"] \
		* seg_rest
	for i in range(1, n_seg):
		node_F[i] += Vector2(0, -m_node * P["g"])
		var vr := Vector2(U, 0) - nodes_v[i]
		node_F[i] += cd_cyl * vr.length() * vr

	# pod rides the line: walk arc length back from the kite end
	var arc_total := 0.0
	for i in n_seg:
		arc_total += (nodes_p[i + 1] - nodes_p[i]).length()
	var standoff: float = clampf(arc_total - 3.0, 0.5,
		float(P["pod_standoff"]))
	var is_docked := standoff < float(P["pod_standoff"]) - 0.01
	var remaining := standoff
	var pod_p := nodes_p[0]
	var pod_v := Vector2.ZERO
	var pod_i := 0
	var pod_w := 0.0
	for j in range(n_seg, 0, -1):
		var seg_len := (nodes_p[j] - nodes_p[j - 1]).length()
		if remaining <= seg_len or j == 1:
			var f: float = clampf(remaining / max(seg_len, 1e-9), 0.0, 1.0)
			pod_p = nodes_p[j].lerp(nodes_p[j - 1], f)
			pod_v = nodes_v[j].lerp(nodes_v[j - 1], f)
			pod_i = j
			pod_w = f
			break
		remaining -= seg_len

	# per-node mass, so the pod can actually carry something: the payload
	# is a point mass on the line, split over the two nodes it sits
	# between (same weighting as its force reaction).
	var node_m: Array[float] = []
	node_m.resize(n_seg + 1)
	for i in n_seg + 1:
		node_m[i] = m_node
	if payload_on_pod and payload_kg > 0.0:
		var w_hi: float = 1.0 - pod_w
		node_m[pod_i] += payload_kg * w_hi
		node_m[pod_i - 1] += payload_kg * pod_w
		var wt := Vector2(0.0, -payload_kg * float(P["g"]))
		node_F[pod_i] += wt * w_hi
		node_F[pod_i - 1] += wt * pod_w

	# ---- kite forces (mirror of _derivs, main spring replaced) ----------
	var th := s[2]
	var wa := Vector2(U - s[3], -s[4])
	var va: float = max(wa.length(), 1e-6)
	var alpha := rad_to_deg(th + atan2(wa.y, wa.x))
	var co := _aero(alpha)
	var cl := co.x
	var cd := co.y
	var cm := co.z
	var S: float = P["S"]
	var c_ref: float = P["c_ref"]
	var q: float = 0.5 * P["rho"] * va * va
	var dhat := wa / va
	var lhat := Vector2(-dhat.y, dhat.x)
	var F := q * S * (cd * dhat + cl * lhat)
	var M: float = q * S * c_ref \
		* (cm + P["cm_q"] * s[5] * c_ref / (2.0 * va))
	for pair in [[P["r_skin"], -P["m_skin"] * float(P["g"])],
				 [P["p_main"], -P["m_pod"] * float(P["g"])],
				 [P["r_cb"], float(P["buoyancy_n"])]]:
		var rw := _rot(pair[0], th)
		F.y += pair[1]
		M += -rw.x * pair[1]
	# main line force on the kite = last segment
	var t_main: float = seg_T[n_seg - 1]
	var f_line := -seg_T[n_seg - 1] * seg_dir[n_seg - 1]
	F += f_line
	M += rw_m.y * f_line.x - rw_m.x * f_line.y
	# control pair to the pod
	var k_ctl: float = ea_ctl / sqrt(standoff * standoff + ctl_rest2)
	var rw_c := _rot(P["p_ctl"], th)
	var r_c := Vector2(s[0], s[1]) + rw_c
	var v_c := Vector2(s[3], s[4]) + s[5] * Vector2(rw_c.y, -rw_c.x)
	var v := pod_p - r_c
	var v_len := v.length()
	var dist3: float = sqrt(v_len * v_len + P["y_ctl"] * P["y_ctl"])
	var t_ctl := 0.0
	var uvc := v / maxf(v_len, 1e-9)
	if is_docked:
		# a docked drum holds constant tension however the kite moves —
		# that is what "auto-tend" means. Modelling it as a spring around
		# a shifting rest length let the line damping cancel it, leaving
		# the pair dead slack exactly when attitude authority is needed.
		t_ctl = T_TEND
	elif dist3 > l0c:
		t_ctl = maxf(k_ctl * (dist3 - l0c)
			- P["c_line"] * (v_c - pod_v).dot(uvc), 0.0)
	if t_ctl > 0.0:
		var fc := t_ctl * (v_len / dist3) * uvc
		F += fc
		M += rw_c.y * fc.x - rw_c.x * fc.y
		# reaction on the line nodes the pod sits between
		node_F[pod_i] -= fc * (1.0 - pod_w)
		node_F[pod_i - 1] -= fc * pod_w
	var rcg := _rot(P["r_cg"], th)
	var m_cg: float = M - (rcg.y * F.x - rcg.x * F.y)

	# ---- integrate (semi-implicit Euler) --------------------------------
	for i in range(1, n_seg):
		nodes_v[i] += node_F[i] / node_m[i] * dt
		nodes_p[i] += nodes_v[i] * dt
	var acc := [F.x / (P["m_total"] + P["m_added_x"]),
				F.y / (P["m_total"] + P["m_added_z"]),
				m_cg / (P["i_yy"] + P["i_added"])]
	s[3] += acc[0] * dt
	s[4] += acc[1] * dt
	s[5] += acc[2] * dt
	s[0] += s[3] * dt
	s[1] += s[4] * dt
	s[2] += s[5] * dt

	return {
		"alpha": alpha, "t_main": t_main, "t_ctl": t_ctl,
		"r_m": attach, "r_c": r_c, "pod": pod_p,
		"docked": is_docked, "dist": arc_total,
	}


func _madd(a: Array[float], b: Array, f: float) -> Array[float]:
	var r: Array[float] = []
	for i in 6:
		r.append(a[i] + b[i] * f)
	return r


# ===========================================================================
# Headless runs
# ===========================================================================

## Mirrors tools/export_mkv_replay.py: locked winches, gust at t=15..21,
## 45 s, recorded every 0.1 s. Compare against renders/mkv_replay.csv.
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


## Full recovery, 400 -> 20 m at 5 m/s wind. The procedure the sim forced
## us into (each piece fixes an observed failure):
##   - tension-governed main reel (speed steps pogo the elastic line)
##   - ALPHA-hold on the winchlet, not theta-hold: descending at 2 m/s in
##     5 m/s wind adds ~22 deg of inflow, so the kite must trim nose-down
##     while descending and re-trim level for the hover
##   - stop at 20 m line: the concept's buoyant capture hover
func _run_recovery_test() -> void:
	wind_base = 5.0          # benign-conditions capture, per the concept
	var f := FileAccess.open(out_path, FileAccess.WRITE)
	f.store_line("t,l0m,alpha,theta_deg,z,T_main,T_ctl,docked")
	var dt := 0.004
	var rec := 0.0
	var t_last := 0.0
	var a_last := 14.0
	while sim_t < 300.0:
		var reeling := sim_t > 10.0 and l0m > 20.0
		if reeling:
			# tension-governed reel: full rate below 6 kN, stop above 12
			var gov := clampf((12e3 - t_last) / 6e3, 0.0, 1.0)
			l0m = max(l0m - MAIN_WINCH_RATE * gov * dt, 20.0)
		# alpha-hold winchlet: 6 deg on the way down, 3 deg for the hover
		var a_ref := 6.0 if reeling else 3.0
		l0c = clampf(l0c + 0.08 * (a_last - a_ref) * dt,
			l0c_trim - 3.0, l0c_trim + 3.0)
		var out := _step(dt)
		t_last = out["t_main"]
		a_last = out["alpha"]
		if sim_t >= rec:
			f.store_line("%.2f,%.1f,%.2f,%.2f,%.2f,%.0f,%.0f,%d"
				% [sim_t, l0m, out["alpha"], rad_to_deg(s[2]), s[1],
				   out["t_main"], out["t_ctl"], int(out["docked"])])
			rec += 1.0
		if s[1] < 0.0:
			print("DITCHED at t=%.1f" % sim_t)
			break
		if l0m <= 20.0 and sim_t > 240.0:
			break
	f.close()
	print("recovery test written: " + out_path)
	get_tree().quit()


## "Just hang there": locked winches, segmented line, 90 s with a gust at
## t=40. Honours --wind / --payload / --payload-on-pod, so it answers
## configuration questions the parity gate deliberately refuses to.
func _run_fly_test() -> void:
	var f := FileAccess.open(out_path, FileAccess.WRITE)
	f.store_line("t,U,alpha,theta_deg,x,z,T_main,T_ctl")
	gust_t0 = 40.0
	var dt := 0.004
	var rec := 0.0
	while sim_t < 90.0:
		var out := _step(dt)
		if sim_t >= rec:
			f.store_line("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f"
				% [sim_t, wind_now(), out["alpha"], rad_to_deg(s[2]),
				   s[0], s[1], out["t_main"], out["t_ctl"]])
			rec += 0.25
		if s[1] < 1.0:
			print("DITCHED at t=%.1f" % sim_t)
			break
	f.close()
	print("fly test written: %s  (payload %.0f kg on %s, wind %.1f)"
		% [out_path, payload_kg, "pod" if payload_on_pod else "kite",
		   wind_base])
	get_tree().quit()


func _run_demo() -> void:
	# scripted: settle, gust, winch out (depower), winch in, wind up
	var script := [
		[3.0, ""], [1.0, "gust"], [10.0, ""], [3.0, "winch_out"],
		[6.0, ""], [3.0, "winch_in"], [6.0, ""],
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
			hud.show_state(_hud_state(1.0 / fps))
			await get_tree().process_frame
			await RenderingServer.frame_post_draw
			var img: Image = get_viewport().get_texture().get_image()
			img.save_png("%s/frame_%04d.png" % [out_path, demo_frame])
			demo_frame += 1
			t_seg += 1.0 / fps
	print("demo captured %d frames" % demo_frame)
	get_tree().quit()


## Godot would otherwise import capture output as game assets (and parse
## stray CSVs as translations) — keep the scratch dirs out of the project.
func _ensure_out_dir(path: String) -> void:
	DirAccess.make_dir_recursive_absolute(path)
	if not FileAccess.file_exists(path + "/.gdignore"):
		var f := FileAccess.open(path + "/.gdignore", FileAccess.WRITE)
		if f:
			f.close()


# ===========================================================================
# Interactive
# ===========================================================================

func _physics_process(delta: float) -> void:
	if mode != "interactive":
		return
	if not splashed and s[1] < 2.0:
		splashed = true
		paused = true
	if paused:
		hud.show_state(_hud_state(delta))
		return
	_handle_input(delta)
	var want := int(round(delta * time_scale() / DT))
	var out: Dictionary
	for i in clampi(want, 1, MAX_STEPS_PER_FRAME):
		out = _step(DT)
	_update_visuals(out)
	hud.show_state(_hud_state(delta))


func _handle_input(delta: float) -> void:
	if Input.is_action_pressed("sim_wind_up"):
		wind_base = clampf(wind_base + 2.0 * delta, 2.0, 26.0)
	if Input.is_action_pressed("sim_wind_down"):
		wind_base = clampf(wind_base - 2.0 * delta, 2.0, 26.0)
	var manual := Input.is_action_pressed("sim_trim_in") \
		or Input.is_action_pressed("sim_trim_out")
	if Input.is_action_pressed("sim_trim_in"):
		l0c = clampf(l0c - WINCH_RATE * delta, l0c_trim - 3.0, l0c_trim + 3.0)
	if Input.is_action_pressed("sim_trim_out"):
		l0c = clampf(l0c + WINCH_RATE * delta, l0c_trim - 3.0, l0c_trim + 3.0)
	# ALPHA-hold, not attitude-hold: hauling the kite in adds inflow, so
	# holding theta lets alpha run away — it climbs, overflies the ship
	# and noses over. This is the same law the recovery procedure needed
	# (godot/README.md); holding alpha, winching in just descends.
	if trim_hold and not manual:
		var err: float = float(last_out.get("alpha", alpha_hold_deg)) \
			- alpha_hold_deg
		l0c = clampf(l0c + ALPHA_HOLD_GAIN * err * delta,
			l0c_trim - 3.0, l0c_trim + 3.0)
	if Input.is_action_pressed("sim_payload_up"):
		_apply_payload(payload_kg + 120.0 * delta)
	if Input.is_action_pressed("sim_payload_down"):
		_apply_payload(payload_kg - 120.0 * delta)
	var main_rate := MAIN_WINCH_RATE \
		* (5.0 if Input.is_action_pressed("sim_winch_fast") else 1.0)
	if Input.is_action_pressed("sim_winch_in"):
		l0m = clampf(l0m - main_rate * delta, 8.0, l0m_full)
	if Input.is_action_pressed("sim_winch_out"):
		l0m = clampf(l0m + main_rate * delta, 8.0, l0m_full)


func _unhandled_input(event: InputEvent) -> void:
	if mode != "interactive":
		return
	if cam and cam.handle_input(event):
		return
	if event.is_action_pressed("sim_gust"):
		gust_t0 = sim_t
	elif event.is_action_pressed("sim_reset"):
		_reset()
	elif event.is_action_pressed("sim_pause"):
		paused = not paused
	elif event.is_action_pressed("sim_trim_hold"):
		trim_hold = not trim_hold
		if trim_hold:      # capture whatever you are flying right now
			alpha_hold_deg = clampf(
				float(last_out.get("alpha", 6.0)), -2.0, 16.0)
	elif event.is_action_pressed("sim_time_slower"):
		rate_idx = maxi(rate_idx - 1, 0)
	elif event.is_action_pressed("sim_time_faster"):
		rate_idx = mini(rate_idx + 1, TIME_RATES.size() - 1)
	elif event.is_action_pressed("sim_payload_where"):
		payload_on_pod = not payload_on_pod
		if payload_on_pod:
			seg_mode = true          # needs the line to hang from
		_apply_payload(payload_kg)
	elif event.is_action_pressed("sim_camera"):
		cam.cycle()
	elif event.is_action_pressed("sim_help"):
		hud.toggle_legend()
	elif event.is_action_pressed("sim_quit"):
		get_tree().quit()
	# Manta type switching
	elif event.is_action_pressed("manta_next"):
		manta_type_loader.switch_to_next()
	elif event.is_action_pressed("manta_prev"):
		manta_type_loader.switch_to_previous()
	elif event.is_action_pressed("manta_menu"):
		# For now, just cycle to next - full menu would need UI
		manta_type_loader.switch_to_next()
	else:
		for i in SCENARIOS.size():
			if event.is_action_pressed("sim_scenario_%d" % (i + 1)):
				scenario = i
				_reset()
				return


func _hud_state(dt_real: float) -> Dictionary:
	var out := last_out
	return {
		"t": sim_t, "wind": wind_now(),
		"alpha": out.get("alpha", 0.0), "theta": rad_to_deg(s[2]),
		"alt": s[1], "line": l0m, "trim": l0c - l0c_trim,
		"t_main": out.get("t_main", 0.0), "t_ctl": out.get("t_ctl", 0.0),
		"payload": payload_kg, "net_lift": net_lift_kg(),
		"payload_where": "pod" if payload_on_pod else "kite",
		"hold_alpha": alpha_hold_deg,
		"extrap": _aero_extrapolated(out.get("alpha", 0.0)),
		"wll": P["wll_n"], "ctl_cap": P["ctl_cap_n"],
		"docked": docked, "hold": trim_hold, "paused": paused,
		"splash": splashed, "gust": (sim_t - gust_t0) <= 6.0,
		"time_scale": time_scale(), "n_seg": n_seg,
		"scenario": SCENARIOS[scenario]["name"],
		"camera": cam.mode_name(), "dt_real": dt_real,
		"manta_type": current_manta_type,
	}


# ===========================================================================
# Scene
# ===========================================================================

func _build() -> void:
	get_window().title = "Manta — Mk V live sim"
	add_child(KytoonWorld.environment())
	KytoonWorld.lights(self)
	add_child(KytoonWorld.sea())
	add_child(KytoonWorld.ship())

	kite = KytoonWorld.kite("mkv", KytoonWorld.MK_COLOR.get(current_manta_type, KytoonWorld.MK_COLOR["B"]))
	if kite == null:
		kite = Node3D.new()
	var holder := Node3D.new()
	holder.add_child(kite)
	add_child(holder)
	kite = holder

	mat_main = StandardMaterial3D.new()
	mat_main.roughness = 0.9
	mat_ctl = StandardMaterial3D.new()
	mat_ctl.roughness = 0.9
	line_main = _add_line(0.20, mat_main)
	line_ctl_l = _add_line(0.10, mat_ctl)
	line_ctl_r = _add_line(0.10, mat_ctl)

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

	cam = SimCamera.new()
	add_child(cam)
	hud = SimHud.new()
	add_child(hud)


func _add_line(radius: float, mat: StandardMaterial3D) -> MeshInstance3D:
	var mi := KytoonWorld.line_mesh(radius, mat)
	add_child(mi)
	return mi


func _update_visuals(out: Dictionary) -> void:
	last_out = out
	docked = out["docked"]
	var pos := Vector3(s[0], s[1], 0.0)
	kite.position = pos
	kite.rotation = Vector3(0, 0, -s[2])
	var pod_v: Vector2 = out["pod"]
	var pod_pos := Vector3(pod_v.x, pod_v.y, 0)
	pod.position = pod_pos
	# a laden pod is visibly a gondola, not a winchlet
	var bulk := 1.0 + (2.2 * payload_kg / 800.0 if payload_on_pod else 0.0)
	pod.scale = Vector3(bulk, bulk, bulk)
	var rc: Vector2 = out["r_c"]
	var fairlead := Vector3(P["fairlead"][0], P["fairlead"][1], 0)

	if seg_mode:
		line_main.visible = false
		while seg_lines.size() < n_seg:
			seg_lines.append(_add_line(0.20, mat_main))
		while seg_lines.size() > n_seg:
			seg_lines.pop_back().queue_free()
		for i in n_seg:
			KytoonWorld.stretch(seg_lines[i],
				Vector3(nodes_p[i].x, nodes_p[i].y, 0),
				Vector3(nodes_p[i + 1].x, nodes_p[i + 1].y, 0))
	else:
		var rm: Vector2 = out["r_m"]
		KytoonWorld.stretch(line_main, fairlead, Vector3(rm.x, rm.y, 0))
	KytoonWorld.stretch(line_ctl_l, pod_pos, Vector3(rc.x, rc.y, -P["y_ctl"]))
	KytoonWorld.stretch(line_ctl_r, pod_pos, Vector3(rc.x, rc.y, P["y_ctl"]))
	KytoonWorld.tension_color(mat_main, out["t_main"], P["wll_n"])
	KytoonWorld.tension_color(mat_ctl, out["t_ctl"], P["ctl_cap_n"])

	cam.track(pos, pod_pos, fairlead, s[2])
