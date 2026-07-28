class_name KytoonWorld
extends RefCounted

## Shared scene furniture for every kytoon scene: sky/fog, sun+fill,
## sea, the ship, kite models, and line-segment helpers. Static only —
## call from a scene's builder, e.g.
##
##     add_child(KytoonWorld.environment())
##     KytoonWorld.lights(self)
##
## Entity colors mirror kytoon/viz.py MK_COLOR — fixed per Mk, do not
## re-derive them from series order.

const MK_COLOR := {
	"I": Color("2a78d6"), "II": Color("1baf7a"), "III": Color("eda100"),
	"IV": Color("008300"), "V": Color("4a3aa7"),
}
const SLACK := Color(0.85, 0.85, 0.82)
const LOADED := Color(0.95, 0.15, 0.1)


static func environment(fog_density: float = 0.0004) -> WorldEnvironment:
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
	env.fog_density = fog_density
	var we := WorldEnvironment.new()
	we.environment = env
	return we


static func lights(parent: Node, shadow_distance: float = 800.0) -> void:
	var sun := DirectionalLight3D.new()
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = shadow_distance
	sun.light_energy = 1.8
	sun.light_color = Color(1.0, 0.93, 0.82)
	sun.rotation_degrees = Vector3(-24.0, 35.0, 0.0)
	parent.add_child(sun)
	var fill := DirectionalLight3D.new()
	fill.shadow_enabled = false
	fill.light_energy = 0.35
	fill.light_color = Color(0.6, 0.7, 1.0)
	fill.rotation_degrees = Vector3(-18.0, -140.0, 0.0)
	parent.add_child(fill)


static func sea(size: float = 20000.0) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(size, size)
	mi.mesh = plane
	var mat := StandardMaterial3D.new()
	mat.albedo_color = Color(0.03, 0.10, 0.17)
	mat.metallic = 0.05
	mat.metallic_specular = 0.08
	mat.roughness = 0.6
	mi.material_override = mat
	return mi


## 40 m trimaran (Sea Hunter class) — center hull, outriggers, deckhouse,
## winch pedestal at the origin so the fairlead sits above it.
static func ship() -> Node3D:
	var root := Node3D.new()
	root.name = "Ship"
	var gray := StandardMaterial3D.new()
	gray.albedo_color = Color(0.28, 0.30, 0.34)
	gray.roughness = 0.85
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
		root.add_child(b)
	return root


## Load models/<name>.glb and tint it. Returns null if the export is
## missing (run `python -m kytoon.geometry specs/ -o models` first).
## The exporter authors x-downstream / y-spanwise / z-up while glTF is
## Y-up, hence the -90 deg X rotation. yaw_deg then picks the heading:
## 0 puts the span across X (longitudinal scenes, wind along +X);
## 90 puts the span along X with the nose at -Z (the fleet lineup).
## metallic stays at 0 for the sim scenes; the fleet lineup uses a touch
## of it so the models read against the sky at 175 m.
static func kite(name: String, color: Color, yaw_deg: float = 0.0,
		metallic: float = 0.0) -> Node3D:
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	var path := ProjectSettings.globalize_path("res://") + "../models/%s.glb" % name
	if doc.append_from_file(path, state) != OK:
		push_error("KytoonWorld: cannot load " + path)
		return null
	var wrapper := Node3D.new()
	wrapper.name = name
	wrapper.rotation_degrees = Vector3(-90.0, yaw_deg, 0.0)
	var model := doc.generate_scene(state)
	wrapper.add_child(model)
	var mat := StandardMaterial3D.new()
	mat.albedo_color = color
	mat.roughness = 0.55
	mat.metallic = metallic
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED   # canopies are open surfaces
	_tint(model, mat)
	return wrapper


static func _tint(n: Node, mat: Material) -> void:
	if n is MeshInstance3D:
		n.material_override = mat
	for c in n.get_children():
		_tint(c, mat)


static func line_mesh(radius: float, mat: StandardMaterial3D) -> MeshInstance3D:
	var mi := MeshInstance3D.new()
	var mesh := CylinderMesh.new()
	mesh.top_radius = radius
	mesh.bottom_radius = radius
	mesh.height = 1.0
	mi.mesh = mesh
	mi.material_override = mat
	return mi


## Point a unit-height cylinder from a to b.
static func stretch(mi: MeshInstance3D, a: Vector3, b: Vector3) -> void:
	var d := b - a
	var span := d.length()
	if span < 1e-6:
		mi.visible = false
		return
	mi.visible = true
	(mi.mesh as CylinderMesh).height = span
	var y := d / span
	var x := y.cross(Vector3.FORWARD).normalized()
	if x.length_squared() < 0.5:
		x = y.cross(Vector3.RIGHT).normalized()
	mi.global_transform = Transform3D(Basis(x, y, x.cross(y)), (a + b) * 0.5)


## White (slack) through red (at the working load limit).
static func tension_color(mat: StandardMaterial3D, tension: float,
		cap: float) -> void:
	mat.albedo_color = SLACK.lerp(LOADED, clampf(tension / cap, 0.0, 1.0))
