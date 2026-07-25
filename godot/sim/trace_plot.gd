class_name TracePlot
extends Control

## Rolling strip chart for the sim HUD: a few traces sharing an x axis
## (sim time), each with its own y range and color. Fixed-capacity ring
## buffers, redrawn every frame — no allocation per sample.

const CAPACITY := 900          # samples held (≈60 s at 15 Hz)

var window_s := 60.0
var _traces: Array[Dictionary] = []
var _t := PackedFloat32Array()
var _head := 0
var _count := 0
var _bg := Color(0.04, 0.05, 0.07, 0.72)
var _grid := Color(1, 1, 1, 0.10)
var _axis := Color(1, 1, 1, 0.30)


func _init() -> void:
	_t.resize(CAPACITY)
	custom_minimum_size = Vector2(360, 132)
	mouse_filter = Control.MOUSE_FILTER_IGNORE


## label, color, lo/hi y range, units, and an optional reference line.
func add_trace(label: String, color: Color, lo: float, hi: float,
		units: String, reference := NAN) -> void:
	var d := {
		"label": label, "color": color, "lo": lo, "hi": hi,
		"units": units, "ref": reference, "v": PackedFloat32Array(),
	}
	d["v"].resize(CAPACITY)
	_traces.append(d)


func push(t: float, values: Array) -> void:
	_t[_head] = t
	for i in mini(values.size(), _traces.size()):
		_traces[i]["v"][_head] = float(values[i])
	_head = (_head + 1) % CAPACITY
	_count = mini(_count + 1, CAPACITY)
	queue_redraw()


func clear() -> void:
	_head = 0
	_count = 0
	queue_redraw()


func _draw() -> void:
	var r := Rect2(Vector2.ZERO, size)
	draw_rect(r, _bg)
	var pad_l := 4.0
	var pad_r := 4.0
	var pad_t := 16.0
	var pad_b := 12.0
	var plot := Rect2(pad_l, pad_t, size.x - pad_l - pad_r,
		size.y - pad_t - pad_b)
	for i in 4:
		var y := plot.position.y + plot.size.y * i / 3.0
		draw_line(Vector2(plot.position.x, y),
			Vector2(plot.end.x, y), _grid, 1.0)
	if _count < 2:
		return

	var t_now: float = _t[(_head - 1 + CAPACITY) % CAPACITY]
	var t0 := t_now - window_s
	var font := ThemeDB.fallback_font
	var legend_x := plot.position.x + 4.0

	for tr in _traces:
		var lo: float = tr["lo"]
		var hi: float = tr["hi"]
		# autoscale upward only, so a quiet trace keeps a stable frame
		for k in _count:
			var idx := (_head - 1 - k + CAPACITY) % CAPACITY
			if _t[idx] < t0:
				break
			var v: float = tr["v"][idx]
			lo = minf(lo, v)
			hi = maxf(hi, v)
		var span: float = maxf(hi - lo, 1e-6)

		if not is_nan(float(tr["ref"])):
			var yr: float = plot.end.y - plot.size.y * (float(tr["ref"]) - lo) / span
			if yr > plot.position.y and yr < plot.end.y:
				draw_dashed_line(Vector2(plot.position.x, yr),
					Vector2(plot.end.x, yr), Color(tr["color"], 0.35), 1.0, 4.0)

		var pts := PackedVector2Array()
		for k in range(_count - 1, -1, -1):
			var idx := (_head - 1 - k + CAPACITY) % CAPACITY
			var t: float = _t[idx]
			if t < t0:
				continue
			var x: float = plot.position.x \
				+ plot.size.x * (t - t0) / maxf(window_s, 1e-6)
			var y: float = plot.end.y \
				- plot.size.y * (float(tr["v"][idx]) - lo) / span
			pts.append(Vector2(x, clampf(y, plot.position.y, plot.end.y)))
		if pts.size() > 1:
			draw_polyline(pts, tr["color"], 1.6, true)

		var last: float = tr["v"][(_head - 1 + CAPACITY) % CAPACITY]
		var txt := "%s %.1f%s" % [tr["label"], last, tr["units"]]
		draw_string(font, Vector2(legend_x, 12.0), txt,
			HORIZONTAL_ALIGNMENT_LEFT, -1, 11, tr["color"])
		legend_x += font.get_string_size(txt, HORIZONTAL_ALIGNMENT_LEFT,
			-1, 11).x + 14.0

	draw_line(plot.position + Vector2(0, plot.size.y), plot.end, _axis, 1.0)
	draw_string(font, Vector2(plot.end.x - 46, size.y - 2),
		"%.0f s" % window_s, HORIZONTAL_ALIGNMENT_LEFT, -1, 10,
		Color(1, 1, 1, 0.45))
