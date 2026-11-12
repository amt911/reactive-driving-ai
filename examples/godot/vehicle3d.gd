# Headless 3D smoke test of the rdai GDExtension (scripts/godot-smoke.sh): RdaiDriver drives a real
# Godot VehicleBody3D (four raycast wheels, Godot's own physics) round an ellipse on the XZ plane.
# Exit 0 after one lap on the track; 1 if it leaves the track, stalls or the extension is missing.
# It proves the 3D boundary recipe of docs/ENGINE-INTEGRATION.md: position Vector2(x, z), rotation
# atan2(f.z, f.x) with f = +basis.z, steering sign, and car limits set to what this car can do.
extends SceneTree

const A := 150.0            # ellipse semi-axes (m)
const B := 100.0
const HALF_W := 6.0
const MAX_STEER := 0.5      # rad, VehicleWheel3D steering at steer = 1
const MASS := 1200.0
const ENGINE_FORCE := 6000.0  # N at throttle 1
const BRAKE_FORCE := 60.0     # VehicleBody3D.brake at brake 1
const MAX_T := 150.0

var car: VehicleBody3D
var driver
var track
var dense := PackedVector2Array()
var t := 0.0
var travelled := 0.0
var prev_angle := 0.0
var top := 0.0
var waited := 0

func _initialize() -> void:
	if not ClassDB.class_exists("RdaiDriver"):
		printerr("FAIL: RdaiDriver not registered (extension not loaded)")
		quit(1)
		return
	var centre := PackedVector2Array()
	var w := PackedFloat32Array()
	for k in 48:
		var a := k / 48.0 * TAU
		centre.append(Vector2(A * cos(a), B * sin(a)))
		w.append(HALF_W)
	for k in 720:
		var a := k / 720.0 * TAU
		dense.append(Vector2(A * cos(a), B * sin(a)))
	track = ClassDB.instantiate("RdaiTrack")
	# The envelope the AI plans with. A guess, on the safe side for this VehicleBody3D: it also laps
	# with the GT defaults (26.9 s, top 41 m/s), so it can do more. Measure your own car and set these.
	track.max_lat_accel = 7.0
	track.max_accel = 4.0
	track.max_brake = 7.0
	track.max_speed = 40.0
	if not track.build(centre, w, w):
		printerr("FAIL: track did not build")
		quit(1)
		return
	driver = ClassDB.instantiate("RdaiDriver")
	driver.wheelbase = 2.7
	driver.max_steer = MAX_STEER
	driver.length = 4.4
	driver.width = 1.8
	if not driver.setup(track, 7, 0.8, 1.0, 0.5):
		printerr("FAIL: setup refused the geometry")
		quit(1)
		return
	_build_world()
	prev_angle = atan2(0.0, 1.0)

func _build_world() -> void:
	var floor_body := StaticBody3D.new()
	var floor_shape := CollisionShape3D.new()
	var box := BoxShape3D.new()
	box.size = Vector3(2.0 * A + 100.0, 1.0, 2.0 * B + 100.0)
	floor_shape.shape = box
	floor_shape.position = Vector3(0, -0.5, 0)
	floor_body.add_child(floor_shape)
	root.add_child(floor_body)

	car = VehicleBody3D.new()
	car.mass = MASS
	var body_shape := CollisionShape3D.new()
	var cbox := BoxShape3D.new()
	cbox.size = Vector3(1.8, 0.6, 4.4)
	body_shape.shape = cbox
	body_shape.position = Vector3(0, 0.5, 0)
	car.add_child(body_shape)
	for z in [1.35, -1.35]:
		for x in [0.8, -0.8]:
			var wheel := VehicleWheel3D.new()
			wheel.position = Vector3(x, 0.3, z)
			wheel.wheel_radius = 0.35
			wheel.wheel_rest_length = 0.3
			wheel.suspension_travel = 0.2
			wheel.suspension_stiffness = 40.0
			wheel.damping_compression = 2.0
			wheel.damping_relaxation = 2.5
			wheel.use_as_steering = z > 0.0
			wheel.use_as_traction = z < 0.0
			car.add_child(wheel)
	# Start on the centreline at (A, 0) facing +Z: VehicleBody3D drives along +basis.z.
	car.position = Vector3(A, 0.4, 0.0)
	root.add_child(car)

func _physics_process(delta: float) -> bool:
	# Nodes added in _initialize enter the tree a frame later.
	if car == null or not car.is_inside_tree():
		waited += 1
		if waited > 10:
			printerr("FAIL: the car never entered the scene tree")
			quit(1)
			return true
		return false
	var p := car.global_position
	var f := car.global_basis.z
	var v := car.linear_velocity.dot(f)
	top = max(top, v)
	var c: Dictionary = driver.tick(Vector2(p.x, p.z), atan2(f.z, f.x), v, PackedFloat32Array(), delta)
	if c.is_empty():
		printerr("FAIL: tick returned nothing at t = %.2f" % t)
		quit(1)
		return true
	car.steering = c["steer"] * MAX_STEER
	car.engine_force = c["throttle"] * ENGINE_FORCE
	car.brake = c["brake"] * BRAKE_FORCE
	var q := Vector2(p.x, p.z)
	var off := INF
	for d in dense:
		off = min(off, q.distance_to(d))
	if off > HALF_W:
		printerr("FAIL: off the track by %.1f m at t = %.2f (v %.1f m/s, steer %.2f)" % [off - HALF_W, t, v, c["steer"]])
		quit(1)
		return true
	var ang := atan2(q.y / B, q.x / A)
	travelled += wrapf(ang - prev_angle, -PI, PI)
	prev_angle = ang
	t += delta
	if travelled >= TAU:
		print("ok: 3D lap in %.2f s (ideal %.2f s), top %.1f m/s" % [t, track.ideal_lap(), top])
		quit(0)
		return true
	if t > MAX_T:
		printerr("FAIL: no 3D lap in %.0f s (travelled %.2f rad)" % [MAX_T, travelled])
		quit(1)
		return true
	return false
