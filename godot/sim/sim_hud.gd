class_name SimHud
extends CanvasLayer

## Instrument panel for the Manta sim. Built in code (the whole project
## builds scenes in code) but with a single public entry point:
##
##     hud.show_state({...})     # see mkv_sim.gd _hud_state()
##
## Layout: readout panel top-left, status pills under it, trace plot
## bottom-left, key legend bottom-right (H toggles).

const INK := Color(0.93, 0.94, 0.96)
const DIM := Color(0.62, 0.66, 0.72)
const GOOD := Color(0.42, 0.85, 0.55)
const WARN := Color(0.95, 0.72, 0.22)
const BAD := Color(0.86, 0.28, 0.24)
const ACCENT := Color(0.55, 0.62, 0.95)

var _rows := {}
var _bars := {}
var _pills := {}
var _title: Label
var _pill_box: HBoxContainer
var _legend: PanelContainer
var _banner: Label
var plot: TracePlot
var _plot_accum := 0.0


func _ready() -> void:
	layer = 10
	var root := MarginContainer.new()
	root.set_anchors_preset(Control.PRESET_FULL_RECT)
	root.add_theme_constant_override("margin_left", 18)
	root.add_theme_constant_override("margin_top", 14)
	root.add_theme_constant_override("margin_right", 18)
	root.add_theme_constant_override("margin_bottom", 14)
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(root)

	var cols := HBoxContainer.new()
	cols.set_anchors_preset(Control.PRESET_FULL_RECT)
	cols.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root.add_child(cols)

	var left := VBoxContainer.new()
	left.add_theme_constant_override("separation", 8)
	left.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	left.mouse_filter = Control.MOUSE_FILTER_IGNORE
	cols.add_child(left)

	left.add_child(_build_readouts())
	_pill_box = HBoxContainer.new()
	_pill_box.add_theme_constant_override("separation", 6)
	left.add_child(_pill_box)
	for id in ["PAUSED", "GUST", "POD DOCKED", "TRIM HOLD", "MAIN SLACK",
			   "SPLASH"]:
		_pills[id] = _make_pill(id)
		_pill_box.add_child(_pills[id])

	var spacer := Control.new()
	spacer.size_flags_vertical = Control.SIZE_EXPAND_FILL
	spacer.mouse_filter = Control.MOUSE_FILTER_IGNORE
	left.add_child(spacer)

	plot = TracePlot.new()
	plot.add_trace("alpha", ACCENT, 0.0, 16.0, "°", 14.0)
	plot.add_trace("T_main", WARN, 0.0, 30.0, " kN")
	left.add_child(plot)

	var right := VBoxContainer.new()
	right.alignment = BoxContainer.ALIGNMENT_END
	right.mouse_filter = Control.MOUSE_FILTER_IGNORE
	cols.add_child(right)
	_legend = _build_legend()
	right.add_child(_legend)

	# straight on the CanvasLayer, not in the MarginContainer, so the
	# container can't restretch it over the readouts
	_banner = Label.new()
	_banner.set_anchors_preset(Control.PRESET_TOP_WIDE)
	_banner.offset_top = 96
	_banner.offset_bottom = 140
	_banner.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_banner.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_banner.add_theme_font_size_override("font_size", 26)
	_banner.add_theme_color_override("font_color", BAD)
	_banner.add_theme_color_override("font_outline_color", Color(0, 0, 0, 0.9))
	_banner.add_theme_constant_override("outline_size", 8)
	_banner.visible = false
	add_child(_banner)


# --- construction -----------------------------------------------------------

func _panel(alpha: float = 0.62) -> PanelContainer:
	var p := PanelContainer.new()
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(0.05, 0.06, 0.09, alpha)
	sb.corner_radius_top_left = 6
	sb.corner_radius_top_right = 6
	sb.corner_radius_bottom_left = 6
	sb.corner_radius_bottom_right = 6
	sb.content_margin_left = 14
	sb.content_margin_right = 14
	sb.content_margin_top = 10
	sb.content_margin_bottom = 10
	p.add_theme_stylebox_override("panel", sb)
	p.mouse_filter = Control.MOUSE_FILTER_IGNORE
	return p


func _build_readouts() -> PanelContainer:
	var panel := _panel()
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 2)
	panel.add_child(box)

	_title = Label.new()
	_title.text = "Mk V «Manta» — live sim"
	_title.add_theme_font_size_override("font_size", 18)
	_title.add_theme_color_override("font_color", INK)
	box.add_child(_title)

	var sub := Label.new()
	sub.name = "sub"
	sub.text = "GDScript port of l1_trim"
	sub.add_theme_font_size_override("font_size", 11)
	sub.add_theme_color_override("font_color", DIM)
	box.add_child(sub)
	_rows["_sub"] = sub

	box.add_child(_sep())
	_section(box, "FLIGHT")
	_row(box, "alpha", "°")
	_row(box, "theta", "°")
	_row(box, "altitude", "m")
	_row(box, "wind", "m/s")
	box.add_child(_sep())
	_section(box, "LOAD")
	_row(box, "payload", "kg")
	_row(box, "net lift", "kg")
	box.add_child(_sep())
	_section(box, "RIG")
	_row(box, "line out", "m")
	_row(box, "trim", "m")
	_bar_row(box, "T main", "kN")
	_bar_row(box, "T control", "kN")
	box.add_child(_sep())
	_section(box, "SIM")
	_row(box, "time", "s")
	_row(box, "rate", "")
	_row(box, "segments", "")
	return panel


func _sep() -> Control:
	var s := Control.new()
	s.custom_minimum_size = Vector2(0, 7)
	s.mouse_filter = Control.MOUSE_FILTER_IGNORE
	return s


func _section(parent: Node, text: String) -> void:
	var l := Label.new()
	l.text = text
	l.add_theme_font_size_override("font_size", 10)
	l.add_theme_color_override("font_color", Color(0.5, 0.56, 0.66))
	parent.add_child(l)


func _row(parent: Node, label: String, units: String) -> void:
	var h := HBoxContainer.new()
	h.add_theme_constant_override("separation", 8)
	var name_l := Label.new()
	name_l.text = label
	name_l.custom_minimum_size = Vector2(84, 0)
	name_l.add_theme_font_size_override("font_size", 14)
	name_l.add_theme_color_override("font_color", DIM)
	h.add_child(name_l)
	var val := Label.new()
	val.text = "—"
	val.custom_minimum_size = Vector2(66, 0)
	val.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	val.add_theme_font_size_override("font_size", 15)
	val.add_theme_color_override("font_color", INK)
	h.add_child(val)
	var unit_l := Label.new()
	unit_l.text = units
	unit_l.custom_minimum_size = Vector2(34, 0)
	unit_l.add_theme_font_size_override("font_size", 11)
	unit_l.add_theme_color_override("font_color", DIM)
	h.add_child(unit_l)
	parent.add_child(h)
	_rows[label] = val


func _bar_row(parent: Node, label: String, units: String) -> void:
	_row(parent, label, units)
	var bar := ProgressBar.new()
	bar.show_percentage = false
	bar.mouse_filter = Control.MOUSE_FILTER_IGNORE   # don't eat camera drag
	bar.custom_minimum_size = Vector2(184, 5)
	bar.max_value = 1.0
	var bg := StyleBoxFlat.new()
	bg.bg_color = Color(1, 1, 1, 0.10)
	bg.corner_radius_top_left = 2
	bg.corner_radius_bottom_right = 2
	bar.add_theme_stylebox_override("background", bg)
	var fg := StyleBoxFlat.new()
	fg.bg_color = GOOD
	fg.corner_radius_top_left = 2
	fg.corner_radius_bottom_right = 2
	bar.add_theme_stylebox_override("fill", fg)
	parent.add_child(bar)
	_bars[label] = {"bar": bar, "style": fg}


func _make_pill(text: String) -> PanelContainer:
	var p := PanelContainer.new()
	var sb := StyleBoxFlat.new()
	sb.bg_color = Color(WARN, 0.85)
	sb.corner_radius_top_left = 9
	sb.corner_radius_top_right = 9
	sb.corner_radius_bottom_left = 9
	sb.corner_radius_bottom_right = 9
	sb.content_margin_left = 9
	sb.content_margin_right = 9
	sb.content_margin_top = 3
	sb.content_margin_bottom = 3
	p.add_theme_stylebox_override("panel", sb)
	p.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var l := Label.new()
	l.text = text
	l.add_theme_font_size_override("font_size", 12)
	l.add_theme_color_override("font_color", Color(0.06, 0.06, 0.08))
	p.add_child(l)
	p.visible = false
	p.set_meta("style", sb)
	return p


func _build_legend() -> PanelContainer:
	var panel := _panel(0.5)
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 1)
	panel.add_child(box)
	for pair in [["Up / Down", "wind"], ["- / =", "payload"],
				 ["W / S", "winchlet trim"],
				 ["T", "trim autohold"], ["I / O", "main winch (Shift ×5)"],
				 ["G", "gust +6 m/s"], ["[ / ]", "time rate"],
				 ["1 / 2 / 3", "scenario"], ["C", "camera"],
				 ["drag / wheel", "orbit / zoom"], ["Space", "pause"],
				 ["R", "reset"], ["H", "hide help"]]:
		var h := HBoxContainer.new()
		h.add_theme_constant_override("separation", 10)
		var k := Label.new()
		k.text = pair[0]
		k.custom_minimum_size = Vector2(84, 0)
		k.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
		k.add_theme_font_size_override("font_size", 12)
		k.add_theme_color_override("font_color", ACCENT)
		h.add_child(k)
		var v := Label.new()
		v.text = pair[1]
		v.custom_minimum_size = Vector2(132, 0)
		v.add_theme_font_size_override("font_size", 12)
		v.add_theme_color_override("font_color", DIM)
		h.add_child(v)
		box.add_child(h)
	return panel


# --- update -----------------------------------------------------------------

func toggle_legend() -> void:
	_legend.visible = not _legend.visible


func show_state(d: Dictionary) -> void:
	_put("alpha", "%.1f" % d["alpha"])
	_put("theta", "%.1f" % d["theta"])
	_put("altitude", "%.0f" % d["alt"])
	_put("wind", "%.1f" % d["wind"])
	_put("payload", "%.0f" % d["payload"])
	_put("net lift", "%+.0f" % d["net_lift"])
	# negative net lift means the airframe no longer floats — it has to be
	# flown, and it sinks the moment the wind drops
	_rows["net lift"].add_theme_color_override("font_color",
		BAD if d["net_lift"] < 0.0 else GOOD)
	_put("line out", "%.0f" % d["line"])
	_put("trim", "%+.2f" % d["trim"])
	_put("T main", "%.1f" % (d["t_main"] / 1e3))
	_put("T control", "%.2f" % (d["t_ctl"] / 1e3))
	_put("time", "%.0f" % d["t"])
	_put("rate", _rate_text(d))
	_put("segments", "%d" % d["n_seg"])
	_rows["_sub"].text = "%s  ·  %s cam  ·  GDScript port of l1_trim" % [
		d["scenario"], d["camera"]]

	_set_bar("T main", d["t_main"] / d["wll"])
	_set_bar("T control", d["t_ctl"] / d["ctl_cap"])

	# alpha readout turns amber approaching the stall shoulder
	var a: float = d["alpha"]
	_rows["alpha"].add_theme_color_override("font_color",
		BAD if a > 20.0 or a < -8.0 else (WARN if a > 16.0 else INK))

	_pill("PAUSED", d["paused"])
	_pill("GUST", d["gust"])
	_pill("POD DOCKED", d["docked"])
	_pill("TRIM HOLD", d["hold"])
	_pill("MAIN SLACK", d["t_main"] < 50.0)
	_pill("SPLASH", d["splash"])

	_banner.visible = d["splash"]
	if d["splash"]:
		_banner.text = "SPLASH — kite in the water.   R to reset"

	_plot_accum += d["dt_real"]
	if _plot_accum >= 1.0 / 15.0:
		_plot_accum = 0.0
		plot.push(d["t"], [d["alpha"], d["t_main"] / 1e3])


## GDScript's % has no %g, so trim by hand: 0.25 -> x0.25, 0.5 -> x0.5.
func _rate_text(d: Dictionary) -> String:
	if d["paused"]:
		return "paused"
	var r: float = d["time_scale"]
	if r < 1.0:
		return ("x%.2f" % r).trim_suffix("0")
	return "x%d" % int(r)


func _put(key: String, text: String) -> void:
	_rows[key].text = text


func _set_bar(key: String, frac: float) -> void:
	var e: Dictionary = _bars[key]
	var f: float = clampf(frac, 0.0, 1.0)
	e["bar"].value = f
	e["style"].bg_color = GOOD.lerp(BAD, clampf((f - 0.35) / 0.4, 0.0, 1.0))


func _pill(id: String, on: bool) -> void:
	_pills[id].visible = on


func reset_plot() -> void:
	plot.clear()
