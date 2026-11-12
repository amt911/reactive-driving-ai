# Headless smoke test of the rdai GDExtension (scripts/godot-smoke.sh): one AI car, driven by
# RdaiDriver, laps an ellipse in Godot 2D conventions (Y down, 10 units per metre) on a
# GDScript kinematic bicycle. Exit 0 after one lap on the track; 1 if it leaves the track,
# stalls, or the extension is missing. A wrong axis, angle or steering sign at the boundary sends
# the car off the track within seconds.
extends SceneTree

const UPM := 10.0          # Godot units per metre
const A := 150.0 * UPM      # ellipse semi-axes
const B := 100.0 * UPM
const HALF_W := 6.0 * UPM
const WHEELBASE := 2.7 * UPM
const MAX_STEER := 0.45
const DT := 1.0 / 60.0

func _initialize() -> void:
	if not ClassDB.class_exists("RdaiDriver"):
		printerr("FAIL: RdaiDriver not registered (extension not loaded)")
		quit(1)
		return
	var centre := PackedVector2Array()
	var wl := PackedFloat32Array()
	var wr := PackedFloat32Array()
	for k in 48:
		var a := k / 48.0 * TAU
		centre.append(Vector2(A * cos(a), B * sin(a)))
		wl.append(HALF_W)
		wr.append(HALF_W)
	var track = ClassDB.instantiate("RdaiTrack")
	track.meters_per_unit = 1.0 / UPM
	if not track.build(centre, wl, wr):
		printerr("FAIL: track did not build")
		quit(1)
		return
	var dense := PackedVector2Array()
	for k in 720:
		var a := k / 720.0 * TAU
		dense.append(Vector2(A * cos(a), B * sin(a)))
	var driver = ClassDB.instantiate("RdaiDriver")
	driver.setup(track, 7, 0.8, 1.0, 0.5)
	var pos := Vector2(A, 0.0)
	var rot := PI / 2.0   # moving along +Y on screen, the direction the points go
	var v := 10.0 * UPM
	var travelled := 0.0
	var prev_angle := atan2(pos.y / B, pos.x / A)
	var t := 0.0
	while t < 120.0:
		var c: Dictionary = driver.tick(pos, rot, v, PackedFloat32Array(), DT)
		if c.is_empty():
			printerr("FAIL: tick returned nothing at t = %.2f" % t)
			quit(1)
			return
		var accel: float = (8.0 * c["throttle"] - 12.0 * c["brake"]) * UPM
		v = max(v + accel * DT, 0.0)
		# steer +1 turns to the driver's left: counter-clockwise on screen, rotation decreasing.
		rot -= v / WHEELBASE * tan(c["steer"] * MAX_STEER) * DT
		pos += Vector2(cos(rot), sin(rot)) * v * DT
		var off := INF
		for p in dense:
			off = min(off, pos.distance_to(p))
		if off > HALF_W:
			printerr("FAIL: off the track by %.1f m at t = %.2f (steer %.2f)" % [(off - HALF_W) / UPM, t, c["steer"]])
			quit(1)
			return
		var ang := atan2(pos.y / B, pos.x / A)
		travelled += wrapf(ang - prev_angle, -PI, PI)
		prev_angle = ang
		t += DT
		if travelled >= TAU:
			print("ok: lap in %.2f s (ideal %.2f s), top %.1f m/s" % [t, track.ideal_lap(), v / UPM])
			quit(0)
			return
	printerr("FAIL: no lap in 120 s (travelled %.2f rad)" % travelled)
	quit(1)
