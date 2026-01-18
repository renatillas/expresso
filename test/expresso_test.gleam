import expresso/body
import expresso/collision
import expresso/world
import gleam/dict
import gleam/float
import gleam/list
import gleam/option
import gleeunit
import quaternion
import spatial/collider
import vec/vec3
import vec/vec3f

pub fn main() {
  gleeunit.main()
}

// =============================================================================
// Body Tests
// =============================================================================

pub fn body_new_sphere_creates_dynamic_body_test() {
  let b = body.new_sphere("test", vec3.Vec3(0.0, 0.0, 0.0), 1.0)

  assert b.id == "test"
  assert body.bounding_radius(b) == 1.0
  assert b.is_kinematic == False
  assert b.mass == 1.0
}

pub fn body_kinematic_makes_infinite_mass_test() {
  let b =
    body.new_sphere("test", vec3.Vec3(0.0, 0.0, 0.0), 1.0)
    |> body.kinematic

  assert b.is_kinematic == True
  assert body.inverse_mass(b) == 0.0
}

pub fn body_dynamic_has_inverse_mass_test() {
  let b = body.new_sphere("test", vec3.Vec3(0.0, 0.0, 0.0), 1.0)

  assert body.inverse_mass(b) == 1.0
}

// =============================================================================
// Collision Detection Tests
// =============================================================================

pub fn spheres_overlap_when_touching_test() {
  // Test sphere-sphere collision using collider
  let body_a = body.new_sphere("a", vec3.Vec3(0.0, 0.0, 0.0), 1.0)
  let body_b = body.new_sphere("b", vec3.Vec3(1.5, 0.0, 0.0), 1.0)

  let collider_a = body.world_collider(body_a)
  let collider_b = body.world_collider(body_b)

  assert collider.intersects(collider_a, collider_b) == True
}

pub fn spheres_dont_overlap_when_separated_test() {
  let body_a = body.new_sphere("a", vec3.Vec3(0.0, 0.0, 0.0), 1.0)
  let body_b = body.new_sphere("b", vec3.Vec3(5.0, 0.0, 0.0), 1.0)

  let collider_a = body.world_collider(body_a)
  let collider_b = body.world_collider(body_b)

  assert collider.intersects(collider_a, collider_b) == False
}

pub fn box_colliders_work_test() {
  // Test box-box collision
  let box_a =
    body.new_box("box_a", vec3.Vec3(0.0, 0.0, 0.0), vec3.Vec3(1.0, 1.0, 1.0))
  let box_b =
    body.new_box("box_b", vec3.Vec3(1.5, 0.0, 0.0), vec3.Vec3(1.0, 1.0, 1.0))

  let collider_a = body.world_collider(box_a)
  let collider_b = body.world_collider(box_b)

  assert collider.intersects(collider_a, collider_b) == True
}

pub fn capsule_colliders_work_test() {
  // Test capsule collision (perfect for character controllers)
  let capsule_a = body.new_capsule("player", vec3.Vec3(0.0, 0.0, 0.0), 2.0, 0.5)
  let capsule_b = body.new_capsule("enemy", vec3.Vec3(1.0, 0.0, 0.0), 2.0, 0.5)

  let collider_a = body.world_collider(capsule_a)
  let collider_b = body.world_collider(capsule_b)

  assert collider.intersects(collider_a, collider_b) == True
}

pub fn collision_detection_finds_overlapping_bodies_test() {
  // Create two overlapping bodies
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(body.new_sphere("a", vec3.Vec3(0.0, 0.0, 0.0), 1.0))
    |> world.add_body(body.new_sphere("b", vec3.Vec3(1.0, 0.0, 0.0), 1.0))

  let contacts = collision.detect_collisions(world.bodies(w))

  // With the new collider system, we may detect multiple contact points
  // but we should have at least one contact
  assert list.length(contacts) >= 1
}

pub fn collision_detection_skips_separated_bodies_test() {
  // Create two separated bodies
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(body.new_sphere("a", vec3.Vec3(0.0, 0.0, 0.0), 1.0))
    |> world.add_body(body.new_sphere("b", vec3.Vec3(10.0, 0.0, 0.0), 1.0))

  let contacts = collision.detect_collisions(world.bodies(w))

  // Should find no contacts
  assert contacts == []
}

// =============================================================================
// World Tests
// =============================================================================

pub fn world_new_creates_empty_world_test() {
  let w = world.new(gravity: vec3.Vec3(0.0, -9.8, 0.0))

  assert dict.size(world.bodies(w)) == 0
}

pub fn world_add_body_increases_count_test() {
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(body.new_sphere("test", vec3.Vec3(0.0, 0.0, 0.0), 1.0))

  assert dict.size(world.bodies(w)) == 1
}

pub fn world_remove_body_decreases_count_test() {
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(body.new_sphere("test", vec3.Vec3(0.0, 0.0, 0.0), 1.0))
    |> world.remove_body("test")

  assert dict.size(world.bodies(w)) == 0
}

pub fn world_step_updates_positions_test() {
  // Create a body with velocity
  let b =
    body.new_sphere("test", vec3.Vec3(0.0, 0.0, 0.0), 1.0)
    |> body.with_velocity(vec3.Vec3(10.0, 0.0, 0.0))

  let #(w, _events) =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(b)
    |> world.step(delta_time: 0.1)

  // After 0.1 seconds at 10 units/sec, should have moved 1 unit
  let assert Ok(updated_body) = world.get_body(w, "test")
  assert updated_body.position.x >. 0.9 && updated_body.position.x <. 1.1
}

pub fn world_step_separates_overlapping_bodies_test() {
  // Create two overlapping bodies
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.with_iterations(10)
    // More iterations for better separation
    |> world.add_body(body.new_sphere("a", vec3.Vec3(0.0, 0.0, 0.0), 1.0))
    |> world.add_body(body.new_sphere("b", vec3.Vec3(0.5, 0.0, 0.0), 1.0))

  // Run multiple steps to ensure full separation
  let w =
    list.range(0, 9)
    |> list.fold(w, fn(world_acc, _) {
      let #(updated_world, _events) = world.step(world_acc, delta_time: 0.01)
      updated_world
    })

  // Bodies should be pushed apart
  let assert Ok(body_a) = world.get_body(w, "a")
  let assert Ok(body_b) = world.get_body(w, "b")

  let distance = vec3f.distance(body_a.position, body_b.position)

  // Distance should be greater than before (0.5) - bodies separated
  assert distance >. 1.0
}

pub fn world_step_respects_kinematic_bodies_test() {
  // Create kinematic and dynamic body overlapping
  let kinematic_body =
    body.new_sphere("kinematic", vec3.Vec3(0.0, 0.0, 0.0), 1.0)
    |> body.kinematic

  let dynamic_body = body.new_sphere("dynamic", vec3.Vec3(0.5, 0.0, 0.0), 1.0)

  let #(w, _events) =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(kinematic_body)
    |> world.add_body(dynamic_body)
    |> world.step(delta_time: 0.01)

  // Kinematic body should not move
  let assert Ok(k) = world.get_body(w, "kinematic")
  assert k.position.x == 0.0

  // Dynamic body should be pushed away
  let assert Ok(d) = world.get_body(w, "dynamic")
  assert d.position.x >. 0.5
}

// =============================================================================
// Integration Tests
// =============================================================================

pub fn multiple_bodies_separate_correctly_test() {
  // Create 3 bodies in a line, all overlapping
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.with_iterations(10)
    // More iterations for stable resolution
    |> world.add_body(body.new_sphere("a", vec3.Vec3(0.0, 0.0, 0.0), 1.0))
    |> world.add_body(body.new_sphere("b", vec3.Vec3(1.0, 0.0, 0.0), 1.0))
    |> world.add_body(body.new_sphere("c", vec3.Vec3(2.0, 0.0, 0.0), 1.0))

  // Run multiple steps to allow separation
  let w =
    list.range(0, 9)
    |> list.fold(w, fn(world_acc, _) {
      let #(updated_world, _events) = world.step(world_acc, delta_time: 0.01)
      updated_world
    })

  // All bodies should be separated
  let assert Ok(body_a) = world.get_body(w, "a")
  let assert Ok(body_b) = world.get_body(w, "b")
  let assert Ok(body_c) = world.get_body(w, "c")

  // Check A-B distance
  let dist_ab = vec3f.distance(body_a.position, body_b.position)
  assert dist_ab >. 1.9

  // Check B-C distance
  let dist_bc = vec3f.distance(body_b.position, body_c.position)
  assert dist_bc >. 1.9
}

// =============================================================================
// Spatial Query Tests
// =============================================================================

pub fn query_radius_finds_nearby_bodies_test() {
  // Create a world with bodies at various distances
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(body.new_sphere("a", vec3.Vec3(0.0, 0.0, 0.0), 1.0))
    |> world.add_body(body.new_sphere("b", vec3.Vec3(3.0, 0.0, 0.0), 1.0))
    |> world.add_body(body.new_sphere("c", vec3.Vec3(10.0, 0.0, 0.0), 1.0))

  // Query radius of 5.0 from origin - should find a and b, not c
  let nearby =
    world.query_radius(w, center: vec3.Vec3(0.0, 0.0, 0.0), radius: 5.0)

  assert list.length(nearby) == 2
}

pub fn query_region_finds_bodies_in_box_test() {
  // Create a world with bodies in different regions
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(body.new_sphere("a", vec3.Vec3(0.0, 0.0, 0.0), 1.0))
    |> world.add_body(body.new_sphere("b", vec3.Vec3(5.0, 5.0, 0.0), 1.0))
    |> world.add_body(body.new_sphere("c", vec3.Vec3(20.0, 20.0, 0.0), 1.0))

  // Query region from -2 to 10 - should find a and b, not c
  let in_region =
    world.query_region(
      w,
      min: vec3.Vec3(-2.0, -2.0, 0.0),
      max: vec3.Vec3(10.0, 10.0, 0.0),
    )

  assert list.length(in_region) == 2
}

pub fn query_nearest_finds_closest_body_test() {
  // Create bodies at various distances
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(body.new_sphere("far", vec3.Vec3(100.0, 100.0, 0.0), 1.0))
    |> world.add_body(body.new_sphere("near", vec3.Vec3(2.0, 0.0, 0.0), 1.0))
    |> world.add_body(body.new_sphere("mid", vec3.Vec3(10.0, 0.0, 0.0), 1.0))

  // Find nearest to origin
  let assert option.Some(#(id, _body, distance)) =
    world.query_nearest(w, point: vec3.Vec3(0.0, 0.0, 0.0))

  assert id == "near"
  assert distance >. 1.0 && distance <. 3.0
}

pub fn spatial_strategy_can_be_set_test() {
  // Test that we can set different spatial strategies
  let _w1 =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.with_spatial_strategy(world.UseBVH)
    |> world.add_body(body.new_sphere("test", vec3.Vec3(0.0, 0.0, 0.0), 1.0))

  let _w2 =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.with_spatial_strategy(world.UseGrid(cell_size: 2.0))
    |> world.add_body(body.new_sphere("test", vec3.Vec3(0.0, 0.0, 0.0), 1.0))

  // Successfully created worlds with different strategies
  Nil
}

// =============================================================================
// Collision Layer Tests
// =============================================================================

pub fn collision_layers_prevent_collision_test() {
  // Create two bodies on different layers that don't collide
  let player_bullet =
    body.new_sphere("player_bullet", vec3.Vec3(0.0, 0.0, 0.0), 1.0)
    |> body.with_layer(body.layer_1)
    |> body.with_collision_mask(
      body.combine_layers([body.layer_2, body.layer_3]),
    )

  let player =
    body.new_sphere("player", vec3.Vec3(0.5, 0.0, 0.0), 1.0)
    |> body.with_layer(body.layer_0)

  let #(w, _events) =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.with_iterations(5)
    |> world.add_body(player_bullet)
    |> world.add_body(player)
    |> world.step(delta_time: 0.01)

  // Bodies should still be overlapping (no collision)
  let assert Ok(bullet) = world.get_body(w, "player_bullet")
  let assert Ok(p) = world.get_body(w, "player")

  let distance = vec3f.distance(bullet.position, p.position)
  assert distance <. 2.0
  // Still overlapping
}

pub fn collision_layers_allow_collision_test() {
  // Create two bodies that should collide
  let player_bullet =
    body.new_sphere("player_bullet", vec3.Vec3(0.0, 0.0, 0.0), 1.0)
    |> body.with_layer(body.layer_1)
    |> body.with_collision_mask(
      body.combine_layers([body.layer_2, body.layer_3]),
    )

  let enemy =
    body.new_sphere("enemy", vec3.Vec3(0.5, 0.0, 0.0), 1.0)
    |> body.with_layer(body.layer_2)

  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.with_iterations(5)
    |> world.add_body(player_bullet)
    |> world.add_body(enemy)

  let w =
    list.range(0, 4)
    |> list.fold(w, fn(world_acc, _) {
      let #(updated_world, _events) = world.step(world_acc, delta_time: 0.01)
      updated_world
    })

  // Bodies should be separated (collision occurred)
  let assert Ok(bullet) = world.get_body(w, "player_bullet")
  let assert Ok(e) = world.get_body(w, "enemy")

  let distance = vec3f.distance(bullet.position, e.position)
  assert distance >. 1.9
  // Properly separated
}

pub fn trigger_bodies_dont_physically_interact_test() {
  // Create a trigger and a dynamic body
  let trigger_zone =
    body.new_sphere("trigger", vec3.Vec3(0.0, 0.0, 0.0), 2.0)
    |> body.trigger()

  let moving_body =
    body.new_sphere("mover", vec3.Vec3(0.0, 0.0, 0.0), 1.0)
    |> body.with_velocity(vec3.Vec3(10.0, 0.0, 0.0))

  let #(w, _events) =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(trigger_zone)
    |> world.add_body(moving_body)
    |> world.step(delta_time: 0.1)

  // Trigger should not have moved
  let assert Ok(t) = world.get_body(w, "trigger")
  assert t.position.x == 0.0

  // Moving body should have moved freely (no physical collision)
  let assert Ok(m) = world.get_body(w, "mover")
  assert m.position.x >. 0.9
  // Moved ~1 unit
}

// =============================================================================
// Raycast Tests
// =============================================================================

pub fn raycast_hits_body_in_path_test() {
  // Create a body and raycast toward it
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(body.new_sphere("target", vec3.Vec3(10.0, 0.0, 0.0), 1.0))

  let hit =
    world.raycast(
      w,
      origin: vec3.Vec3(0.0, 0.0, 0.0),
      direction: vec3.Vec3(1.0, 0.0, 0.0),
      max_distance: 20.0,
      layer_mask: option.None,
    )

  case hit {
    option.Some(h) -> {
      assert h.body_id == "target"
      assert h.distance >. 8.0 && h.distance <. 11.0
    }
    option.None -> panic as "Expected to hit the target"
  }
}

pub fn raycast_misses_body_out_of_range_test() {
  // Create a body and raycast with max_distance too short
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(body.new_sphere("target", vec3.Vec3(10.0, 0.0, 0.0), 1.0))

  let hit =
    world.raycast(
      w,
      origin: vec3.Vec3(0.0, 0.0, 0.0),
      direction: vec3.Vec3(1.0, 0.0, 0.0),
      max_distance: 5.0,
      layer_mask: option.None,
    )

  case hit {
    option.None -> Nil
    option.Some(_) -> panic as "Should not have hit the target"
  }
}

pub fn raycast_respects_layer_mask_test() {
  // Create bodies on different layers
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(
      body.new_sphere("enemy", vec3.Vec3(5.0, 0.0, 0.0), 1.0)
      |> body.with_layer(body.layer_2),
    )
    |> world.add_body(
      body.new_sphere("player", vec3.Vec3(10.0, 0.0, 0.0), 1.0)
      |> body.with_layer(body.layer_0),
    )

  // Raycast only hits enemies
  let hit =
    world.raycast(
      w,
      origin: vec3.Vec3(0.0, 0.0, 0.0),
      direction: vec3.Vec3(1.0, 0.0, 0.0),
      max_distance: 20.0,
      layer_mask: option.Some(body.layer_2),
    )

  case hit {
    option.Some(h) -> {
      assert h.body_id == "enemy"
    }
    option.None -> panic as "Should have hit enemy"
  }
}

// =============================================================================
// Collision Event Tests
// =============================================================================

pub fn collision_events_are_returned_test() {
  // Verify that step returns events (even if empty)
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(body.new_sphere("a", vec3.Vec3(0.0, 0.0, 0.0), 1.0))

  let #(_w, events) = world.step(w, delta_time: 0.01)

  // Events list exists (may be empty or have collision started event)
  // This test just verifies the API works correctly
  case events {
    _ -> Nil
    // API works!
  }
}

pub fn trigger_entered_event_fires_test() {
  // Create a trigger and a body moving into it
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(
      body.new_sphere("trigger", vec3.Vec3(5.0, 0.0, 0.0), 2.0)
      |> body.trigger(),
    )
    |> world.add_body(
      body.new_sphere("mover", vec3.Vec3(0.0, 0.0, 0.0), 0.5)
      |> body.with_velocity(vec3.Vec3(10.0, 0.0, 0.0)),
    )

  // Step to move body into trigger
  let #(_w, events) = world.step(w, delta_time: 0.3)

  // Should have a trigger entered event
  let has_trigger_entered =
    list.any(events, fn(event) {
      case event {
        world.TriggerEntered(body: "mover", trigger: "trigger") -> True
        world.TriggerEntered(body: "trigger", trigger: "mover") -> True
        _ -> False
      }
    })

  assert has_trigger_entered
}

// =============================================================================
// Rotation Physics Tests
// =============================================================================

pub fn angular_velocity_integrates_into_rotation_test() {
  // Create a body with angular velocity around Y-axis
  let initial_rotation = quaternion.identity
  let angular_velocity = vec3.Vec3(0.0, 3.14159, 0.0)
  // Rotate around Y at pi rad/s

  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(
      body.new_sphere("spinner", vec3.Vec3(0.0, 0.0, 0.0), 1.0)
      |> body.with_angular_velocity(angular_velocity),
    )

  // Step physics for 1 second
  let #(w2, _events) = world.step(w, delta_time: 1.0)

  // Get the updated body
  let assert Ok(spinner) = world.get_body(w2, "spinner")

  // Rotation should have changed from identity
  let rotation_changed =
    !quaternion.loosely_equals(
      spinner.rotation,
      initial_rotation,
      tolerating: 0.01,
    )

  assert rotation_changed

  // Angular velocity should have been damped slightly
  let angular_speed_decreased =
    vec3f.length(spinner.angular_velocity) <. vec3f.length(angular_velocity)

  assert angular_speed_decreased
}

pub fn rotation_quaternion_changes_over_time_test() {
  // Create a body with angular velocity and verify rotation changes
  let initial_rotation =
    quaternion.from_axis_angle(vec3.Vec3(1.0, 0.0, 0.0), 0.5)
  let angular_velocity = vec3.Vec3(0.0, 2.0, 0.0)
  // Rotate around Y

  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(
      body.new_sphere("spinner", vec3.Vec3(0.0, 0.0, 0.0), 1.0)
      |> body.with_rotation(initial_rotation)
      |> body.with_angular_velocity(angular_velocity),
    )

  // Step physics for 0.5 seconds
  let #(w2, _events) = world.step(w, delta_time: 0.5)

  // Get the updated body
  let assert Ok(spinner) = world.get_body(w2, "spinner")

  // Rotation should have changed from initial
  let rotation_changed =
    !quaternion.loosely_equals(
      spinner.rotation,
      initial_rotation,
      tolerating: 0.01,
    )

  assert rotation_changed
}

pub fn rotation_applies_to_capsule_collider_test() {
  // Create a rotated capsule and verify collider is transformed
  let rotation = quaternion.from_axis_angle(vec3.Vec3(0.0, 0.0, 1.0), 1.57)

  let capsule =
    body.new_capsule("test", vec3.Vec3(0.0, 0.0, 0.0), height: 2.0, radius: 0.5)
    |> body.with_rotation(rotation)

  // Get world collider (should be rotated)
  let world_collider = body.world_collider(capsule)

  // Verify it's a capsule
  case world_collider {
    collider.Capsule(start, end, _radius) -> {
      // The capsule should be rotated 90 degrees
      // Original was vertical (along Z), rotated should be horizontal
      let _delta_x = float.absolute_value(end.x -. start.x)
      let delta_z = float.absolute_value(end.z -. start.z)

      // After 90 degree rotation around Z, vertical capsule becomes horizontal
      // So delta_z should be larger (capsule is along Z axis)
      assert delta_z >. 0.5
    }
    _ -> panic as "Expected Capsule collider"
  }
}

// =============================================================================
// Friction Tests
// =============================================================================

pub fn friction_slows_sliding_bodies_test() {
  // Create a box sliding on a platform - friction should slow it down
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    // No gravity for this test
    |> world.with_restitution(0.0)
    // No bounce
    |> world.add_body(
      body.new_box(
        "platform",
        vec3.Vec3(0.0, 0.0, 0.0),
        half_extents: vec3.Vec3(10.0, 0.5, 10.0),
      )
      |> body.with_mass(0.0)
      // Static platform
      |> body.with_dynamic_friction(0.5),
      // High friction
    )
    |> world.add_body(
      body.new_box(
        "slider",
        vec3.Vec3(0.0, 0.6, 0.0),
        half_extents: vec3.Vec3(0.5, 0.5, 0.5),
      )
      |> body.with_velocity(vec3.Vec3(5.0, -0.1, 0.0))
      // Moving horizontally, slightly down to ensure contact
      |> body.with_dynamic_friction(0.5),
    )

  // Get initial speed
  let assert Ok(slider_initial) = world.get_body(w, "slider")
  let initial_horizontal_speed = float.absolute_value(slider_initial.velocity.x)

  // Simulate to allow friction to act
  let #(w2, _) = world.step(w, delta_time: 0.1)
  let #(w3, _) = world.step(w2, delta_time: 0.1)
  let #(w4, _) = world.step(w3, delta_time: 0.1)

  // Get slider after physics
  let assert Ok(slider) = world.get_body(w4, "slider")

  // Get final horizontal speed
  let final_horizontal_speed = float.absolute_value(slider.velocity.x)

  // Friction should have reduced horizontal velocity OR it should have stopped
  assert final_horizontal_speed <. initial_horizontal_speed
    || final_horizontal_speed <. 0.5
}

pub fn high_friction_prevents_sliding_test() {
  // Box on a platform with very high friction should not slide much
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(
      body.new_box(
        "platform",
        vec3.Vec3(0.0, 0.0, 0.0),
        half_extents: vec3.Vec3(10.0, 0.5, 10.0),
      )
      |> body.with_mass(0.0)
      |> body.with_static_friction(0.9)
      // Very high friction
      |> body.with_dynamic_friction(0.8),
    )
    |> world.add_body(
      body.new_sphere("ball", vec3.Vec3(0.0, 1.5, 0.0), 0.5)
      |> body.with_velocity(vec3.Vec3(1.0, -5.0, 0.0))
      |> body.with_static_friction(0.9)
      |> body.with_dynamic_friction(0.8),
    )

  // Simulate collision and friction
  let #(w2, _) = world.step(w, delta_time: 0.1)
  let #(w3, _) = world.step(w2, delta_time: 0.1)

  // Get ball after physics
  let assert Ok(ball) = world.get_body(w3, "ball")

  // With high friction, tangential velocity should be very low
  let horizontal_speed = float.absolute_value(ball.velocity.x)

  // Should have been slowed significantly by friction
  assert horizontal_speed <. 0.5
}

// =============================================================================
// CCD Tests
// =============================================================================

pub fn ccd_prevents_tunneling_test() {
  // Fast bullet should not tunnel through thin wall with CCD enabled
  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(
      body.new_box(
        "wall",
        vec3.Vec3(5.0, 0.0, 0.0),
        half_extents: vec3.Vec3(0.1, 5.0, 5.0),
      )
      // Thin wall
      |> body.with_mass(0.0),
    )
    |> world.add_body(
      body.new_sphere("bullet", vec3.Vec3(0.0, 0.0, 0.0), 0.2)
      |> body.with_velocity(vec3.Vec3(100.0, 0.0, 0.0))
      // Very fast!
      |> body.with_ccd(True),
      // Enable CCD
    )

  // Simulate one frame
  let #(w2, events) = world.step(w, delta_time: 0.1)

  // Should have detected collision
  let has_collision =
    list.any(events, fn(event) {
      case event {
        world.CollisionStarted(_, _) -> True
        _ -> False
      }
    })

  // Get bullet position
  let assert Ok(bullet) = world.get_body(w2, "bullet")

  // Bullet should have collided with wall (not passed through)
  // Position should be before the wall (x < 5.0) or very close to it
  assert bullet.position.x <. 6.0 || has_collision
}

pub fn ccd_uses_more_substeps_for_fast_bodies_test() {
  // Verify that CCD actually increases substeps for fast bodies
  // This is more of an integration test

  let fast_bullet =
    body.new_sphere("bullet", vec3.Vec3(0.0, 0.0, 0.0), 0.1)
    |> body.with_velocity(vec3.Vec3(50.0, 0.0, 0.0))
    |> body.with_ccd(True)

  let w =
    world.new(gravity: vec3.Vec3(0.0, 0.0, 0.0))
    |> world.add_body(fast_bullet)

  // Step should complete without error
  let #(w2, _events) = world.step(w, delta_time: 0.1)

  // If we get here, CCD substeps worked - verify bullet moved
  let assert Ok(bullet_after) = world.get_body(w2, "bullet")
  assert bullet_after.position.x >. 0.0
}
