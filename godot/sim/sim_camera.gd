class_name SimCamera
extends Camera3D

## Camera rig for the Manta sim. Four framings (C cycles), each with
## user orbit (drag) and zoom (wheel) on top — the old build spun the
## camera off sim time, which is unusable when you are actually flying.
##
## RIG   — kite and winchlet pod together (default)
## KITE  — close on the airframe, follows its attitude loosely
## SHIP  — from the deck, looking up the line
## WIDE  — frames fairlead → kite, distance from line length

enum Mode { RIG, KITE, SHIP, WIDE }
const MODE_NAME := ["rig", "kite", "ship", "wide"]

var mode: int = Mode.RIG
var azimuth := 0.35
var elevation := 0.06
var zoom := 1.0
var _dragging := false


func _init() -> void:
	fov = 50.0
	far = 8000.0
	near = 0.2


func cycle() -> void:
	mode = (mode + 1) % MODE_NAME.size()


func mode_name() -> String:
	return MODE_NAME[mode]


func handle_input(event: InputEvent) -> bool:
	if event is InputEventMouseButton:
		match event.button_index:
			MOUSE_BUTTON_WHEEL_UP:
				zoom = clampf(zoom * 0.88, 0.15, 6.0)
				return true
			MOUSE_BUTTON_WHEEL_DOWN:
				zoom = clampf(zoom * 1.14, 0.15, 6.0)
				return true
			MOUSE_BUTTON_LEFT, MOUSE_BUTTON_RIGHT:
				_dragging = event.pressed
				return true
	elif event is InputEventMouseMotion and _dragging:
		azimuth -= event.relative.x * 0.006
		elevation = clampf(elevation + event.relative.y * 0.004, -0.35, 0.9)
		return true
	return false


## kite/pod/fairlead in world space; theta is kite pitch (rad).
func track(kite: Vector3, pod: Vector3, fairlead: Vector3,
		theta: float) -> void:
	var focus: Vector3
	var dist: float
	match mode:
		Mode.KITE:
			focus = kite
			dist = 46.0
		Mode.SHIP:
			focus = fairlead.lerp(kite, 0.06)
			dist = 30.0
		Mode.WIDE:
			focus = (fairlead + kite) * 0.5
			dist = maxf(fairlead.distance_to(kite) * 0.85, 60.0)
		_:
			focus = (kite + pod) * 0.5
			dist = maxf(kite.distance_to(pod) * 1.25, 70.0)
	dist *= zoom

	if mode == Mode.SHIP:
		# planted on the deck, looking up whatever the line is doing
		position = fairlead + Vector3(-14.0, 6.0, 12.0)
		look_at(kite, Vector3.UP)
		return

	var el := elevation
	if mode == Mode.KITE:
		el += 0.10 * sin(theta)          # a hint of the airframe's attitude
	var offset := Vector3(cos(azimuth) * dist, el * dist * 2.2,
		sin(azimuth) * dist)
	position = focus + offset
	position.y = maxf(position.y, 3.0)   # never dip below the sea
	look_at(focus, Vector3.UP)
