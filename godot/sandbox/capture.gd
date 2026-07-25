extends Node3D

# Frame count and output dir are overridable from the command line:
#   godot --headless-ish -- --frames=90 --out=/path/to/dir
var frames: int = 90
var out_dir: String = "/mnt/user-data/outputs/frames"

const ORBIT_RADIUS := 9.5
const ORBIT_HEIGHT := 4.0

var cam: Camera3D
var sun: DirectionalLight3D
var spinner: MeshInstance3D
var bobber: MeshInstance3D


func _ready() -> void:
	_parse_args()
	DirAccess.make_dir_recursive_absolute(out_dir)
	_build()

	# Let the renderer settle before the first grab.
	await get_tree().process_frame
	await RenderingServer.frame_post_draw

	var t_start := Time.get_ticks_msec()
	for i in frames:
		_pose(float(i) / float(frames))
		await get_tree().process_frame
		await RenderingServer.frame_post_draw

		var img: Image = get_viewport().get_texture().get_image()
		var path := "%s/frame_%03d.png" % [out_dir, i]
		var err := img.save_png(path)
		if err != OK:
			push_error("save failed on frame %d: %d" % [i, err])

		if i == 0:
			# Sanity check: is anything actually being rasterised?
			print("first frame: %dx%d  centre_px=%s" % [
				img.get_width(), img.get_height(),
				str(img.get_pixel(img.get_width() / 2, img.get_height() / 2))
			])
		if i % 10 == 0 and i > 0:
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


func _mat(albedo: Color, rough: float, metal: float = 0.0) -> StandardMaterial3D:
	var m := StandardMaterial3D.new()
	m.albedo_color = albedo
	m.roughness = rough
	m.metallic = metal
	return m


func _add_mesh(mesh: Mesh, pos: Vector3, mat: StandardMaterial3D) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	mi.mesh = mesh
	mi.position = pos
	mi.material_override = mat
	add_child(mi)
	return mi


func _build() -> void:
	# --- environment -------------------------------------------------
	var we := WorldEnvironment.new()
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.07, 0.09, 0.13)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.42, 0.52, 0.68)
	env.ambient_light_energy = 0.35
	env.fog_enabled = true
	env.fog_light_color = Color(0.10, 0.13, 0.18)
	env.fog_density = 0.012
	we.environment = env
	add_child(we)

	# --- camera ------------------------------------------------------
	cam = Camera3D.new()
	cam.fov = 52.0
	cam.far = 200.0
	add_child(cam)

	# --- key light (shadow-casting, so we can prove shadows work) -----
	sun = DirectionalLight3D.new()
	sun.shadow_enabled = true
	sun.light_energy = 1.7
	sun.light_color = Color(1.0, 0.96, 0.88)
	sun.rotation_degrees = Vector3(-42.0, 35.0, 0.0)
	add_child(sun)

	# cool fill from the opposite side, no shadows
	var fill := DirectionalLight3D.new()
	fill.shadow_enabled = false
	fill.light_energy = 0.45
	fill.light_color = Color(0.55, 0.68, 1.0)
	fill.rotation_degrees = Vector3(-20.0, -140.0, 0.0)
	add_child(fill)

	# --- ground ------------------------------------------------------
	var plane := PlaneMesh.new()
	plane.size = Vector2(40.0, 40.0)
	_add_mesh(plane, Vector3.ZERO, _mat(Color(0.26, 0.28, 0.32), 0.85))

	# --- primitives --------------------------------------------------
	var box := BoxMesh.new()
	box.size = Vector3(1.8, 1.8, 1.8)
	var b := _add_mesh(box, Vector3(-2.8, 0.9, 0.4), _mat(Color(0.85, 0.35, 0.28), 0.45))
	b.rotation_degrees.y = 22.0

	var sphere := SphereMesh.new()
	sphere.radius = 1.15
	sphere.height = 2.3
	_add_mesh(sphere, Vector3(0.2, 1.15, 0.0), _mat(Color(0.80, 0.80, 0.84), 0.18, 0.9))

	var torus := TorusMesh.new()
	torus.inner_radius = 0.55
	torus.outer_radius = 1.15
	spinner = _add_mesh(torus, Vector3(3.0, 1.3, 0.2), _mat(Color(0.95, 0.72, 0.22), 0.35, 0.4))

	var cyl := CylinderMesh.new()
	cyl.top_radius = 0.55
	cyl.bottom_radius = 0.55
	cyl.height = 2.6
	bobber = _add_mesh(cyl, Vector3(0.4, 1.3, -3.0), _mat(Color(0.32, 0.68, 0.52), 0.55))


func _pose(t: float) -> void:
	var a := t * TAU

	cam.position = Vector3(cos(a) * ORBIT_RADIUS, ORBIT_HEIGHT, sin(a) * ORBIT_RADIUS)
	cam.look_at(Vector3(0.0, 1.1, 0.0), Vector3.UP)

	spinner.rotation.y = a * 2.0
	spinner.rotation.x = a
	bobber.position.y = 1.3 + sin(a * 2.0) * 0.55

	# swing the sun so the shadows visibly move
	sun.rotation_degrees.y = 35.0 + sin(a) * 55.0
