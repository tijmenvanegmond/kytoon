extends Node
## Manta Type Loader - Loads and switches between different Manta variants
##
## This node manages the Manta type configuration and allows switching
## between Type A-E variants at runtime.

signal manta_type_changed(type_name: String, config: Dictionary)
signal manta_spec_loaded(spec_data: Dictionary)

@export var config_path: String = "res://godot/data/manta_types.json"
@export var default_type: String = "Type B"

var current_type: String = ""
var manta_config: Dictionary = {}
var all_types: Dictionary = {}


func _ready() -> void:
	load_config()
	switch_to_type(default_type)


func load_config() -> void:
	var file = FileAccess.open(config_path, FileAccess.READ)
	if file:
		var json = JSON.new()
		var parse_result = json.parse(file.get_as_text())
		if parse_result == OK:
			manta_config = json.get_data()
			all_types = manta_config.get("manta_types", {})
			default_type = manta_config.get("default_type", "Type B")
			print("Manta Type Loader: Loaded config with ", all_types.size(), " types")
		else:
			printerr("Manta Type Loader: Failed to parse config: ", json.get_error_message())
		file.close()
	else:
		printerr("Manta Type Loader: Could not open config file: ", config_path)


func get_type_names() -> Array:
	return all_types.keys()


func get_type_config(type_name: String) -> Dictionary:
	return all_types.get(type_name, {})


func switch_to_type(type_name: String) -> bool:
	if not all_types.has(type_name):
		printerr("Manta Type Loader: Unknown type: ", type_name)
		return false
	
	current_type = type_name
	var config = all_types[type_name]
	
	# Load the spec file
	var spec_path = config.get("spec_file", "")
	if spec_path != "":
		load_spec_file(spec_path)
	
	# Emit signals
	emit_signal("manta_type_changed", type_name, config)
	print("Manta Type Loader: Switched to ", type_name)
	return true


func switch_to_next() -> bool:
	var type_names = get_type_names()
	var current_index = type_names.find(current_type)
	if current_index == -1:
		current_index = 0
	else:
		current_index = (current_index + 1) % type_names.size()
	return switch_to_type(type_names[current_index])


func switch_to_previous() -> bool:
	var type_names = get_type_names()
	var current_index = type_names.find(current_type)
	if current_index == -1:
		current_index = type_names.size() - 1
	else:
		current_index = (current_index - 1 + type_names.size()) % type_names.size()
	return switch_to_type(type_names[current_index])


func load_spec_file(spec_path: String) -> Dictionary:
	# Try to load YAML spec file
	# Note: Godot doesn't natively support YAML, so we need a custom loader
	# or use a pre-exported JSON version
	
	# For now, emit a signal with the path so the sim can handle it
	var spec_data = {
		"path": spec_path,
		"type": current_type
	}
	emit_signal("manta_spec_loaded", spec_data)
	return spec_data


func get_current_type() -> String:
	return current_type


func get_current_config() -> Dictionary:
	return all_types.get(current_type, {})


func get_display_name() -> String:
	return get_current_config().get("display_name", current_type)


func get_description() -> String:
	return get_current_config().get("description", "")


func get_color() -> Color:
	var hex_color = get_current_config().get("color", "#FFFFFF")
	return Color.from_html(hex_color)


func get_default_payload() -> float:
	return get_current_config().get("default_payload", 0.0)


func get_max_payload() -> float:
	return get_current_config().get("max_payload", 100.0)


func get_tether_length() -> float:
	return get_current_config().get("tether_length", 400.0)


func get_recommended_wind() -> Vector2:
	var wind_range = get_current_config().get("recommended_wind", [5, 25])
	return Vector2(wind_range[0], wind_range[1])
