extends SceneTree
# Drives racing_game's road_stream.gd / building_stream.gd (see extract_ring_ref.sh) through the steps below with fake
# view/director/simulation nodes and prints one C++ row per job, upload, resident tile and known tile after each step.
# The steps, products and upload rule must equal test_ring_streamer.cpp's.

class FakeView extends Node:
	var focus := Vector3.ZERO
	func godot_to_session(_p: Vector3) -> Vector3:
		return focus

class FakeCamera extends Node:
	var global_position := Vector3.ZERO
	var far := 4000.0

class FakeRig extends Node:
	var camera: FakeCamera

class FakeDirector extends Node:
	var rig: FakeRig
	var focus := Vector2.ZERO
	var velocity := Vector2.ZERO
	func active_rig():
		return rig
	func render_focus_valid() -> bool:
		return true
	func render_focus_session() -> Vector2:
		return focus
	func render_velocity_session() -> Vector2:
		return velocity

class FakeSimulation extends Node:
	func get_render_origin_session() -> Vector3:
		return Vector3.ZERO

const E0 := 466000.0
const N0 := 5552000.0
# {focus x, focus y (session), velocity x, y, uploads after the schedule}
const STEPS := [
	[137.3, 411.9, 0.0, 0.0, 9],
	[301.7, 455.2, 12.0, 3.0, 4],
	[1290.4, 610.8, 35.0, 8.0, 12],
	[2470.1, 1580.6, 30.0, 25.0, 12],
	[2502.9, 1601.3, 30.0, 25.0, 0],
	[-1820.6, -2290.3, -1.0, 0.5, 15],
	[6950.2, 4180.7, 50.0, -10.0, 6],
	[7300.9, 4090.4, 50.0, -10.0, 3],
]

func _content(product: int, x: int, y: int) -> bool:
	if product == 0:
		return posmod(x + y, 5) != 0
	return posmod(x * 7 + y, 3) != 0

func _tri() -> Array:
	return [PackedVector3Array([Vector3.ZERO, Vector3.RIGHT, Vector3.FORWARD]), PackedVector3Array([Vector3.UP, Vector3.UP, Vector3.UP])]

func _result(product: int, job: Dictionary) -> Dictionary:
	var key: Vector2i = job.key
	var origin := Vector3(key.x * 1024.0 - E0, key.y * 1024.0 - N0, 0.0)
	var has := _content(product, key.x, key.y)
	if product == 0:
		var r := {"key": key, "origin_session": origin}
		if has:
			var t := _tri()
			r["vertices"] = t[0]
			r["normals"] = t[1]
			r["uv"] = PackedVector2Array([Vector2.ZERO, Vector2.ZERO, Vector2.ZERO])
			r["uv2"] = PackedVector2Array([Vector2.ZERO, Vector2.ZERO, Vector2.ZERO])
			r["colours"] = PackedColorArray([Color.WHITE, Color.WHITE, Color.WHITE])
			r["indices"] = PackedInt32Array([0, 1, 2])
		else:
			r["vertices"] = PackedVector3Array()
		return r
	var empty := [PackedVector3Array(), PackedVector3Array(), PackedColorArray()]
	var near := empty.duplicate(true)
	if has:
		var t := _tri()
		near = [t[0], t[1], PackedColorArray([Color.WHITE, Color.WHITE, Color.WHITE])]
	return {"key": key, "full": bool(job.full), "origin": origin, "near": near if bool(job.full) else empty.duplicate(true), "tall": near}

func _row(product: int, step: int, kind: String, x: int, y: int, a: int, b: int, c: int, v: float) -> void:
	print("{%d, %d, '%s', %d, %d, %d, %d, %d, %.4f}," % [product, step, kind, x, y, a, b, c, v])

func _run(product: int, script_path: String, settings: Dictionary) -> void:
	var stream = load(script_path).new()
	var view := FakeView.new()
	var rig := FakeRig.new()
	rig.camera = FakeCamera.new()
	var director := FakeDirector.new()
	director.rig = rig
	stream.view = view
	stream.director = director
	stream.simulation = FakeSimulation.new()
	stream.settings = settings
	stream.utm_origin = Vector2(E0, N0)
	for s in STEPS.size():
		var st: Array = STEPS[s]
		view.focus = Vector3(st[0], st[1], 0.0)
		director.focus = Vector2(st[0], st[1])
		director.velocity = Vector2(st[2], st[3])
		stream._process_body(0.016)
		var jobs: Array = stream._jobs.duplicate()
		for i in range(jobs.size() - 1, -1, -1):
			var j: Dictionary = jobs[i]
			_row(product, s, "J", j.key.x, j.key.y, 1 if bool(j.get("full", true)) else 0, 0, 0, float(j.distance))
		for u in int(st[4]):
			if stream._jobs.is_empty():
				break
			var job: Dictionary = stream._jobs.pop_back()
			var full := bool(job.get("full", true))
			_row(product, s, "U", job.key.x, job.key.y, 1 if full else 0, 1 if _content(product, job.key.x, job.key.y) else 0, 0, 0.0)
			stream._upload(_result(product, job))
		stream._process_body(0.016)
		var keys: Array = stream._tiles.keys()
		keys.sort_custom(func(a: Vector2i, b: Vector2i) -> bool: return a.y < b.y or (a.y == b.y and a.x < b.x))
		for k in keys:
			var t: Dictionary = stream._tiles[k]
			if product == 0:
				_row(product, s, "T", k.x, k.y, 1, 1 if t.node.visible else 0, 1 if t.node.visible else 0, 0.0)
			else:
				_row(product, s, "T", k.x, k.y, 1 if bool(t.full) else 0, 1 if t.near.visible else 0, 1 if t.tall.visible else 0, 0.0)
		var known: Array = stream._known.keys()
		known.sort_custom(func(a: Vector2i, b: Vector2i) -> bool: return a.y < b.y or (a.y == b.y and a.x < b.x))
		for k in known:
			_row(product, s, "K", k.x, k.y, 1 if bool(stream._known[k]) else 0, 0, 0, 0.0)

func _initialize() -> void:
	_run(0, "res://scripts/road_stream.gd", {"distance_m": 2500.0, "surface_lift_m": 0.018})
	_run(1, "res://scripts/building_stream.gd", {"ordinary_distance_m": 2200.0, "skyline_distance_m": 6000.0,
		"skyline_min_height_m": 50.0, "fallback_height_m": 8.0, "storey_height_m": 3.0})
	quit()
