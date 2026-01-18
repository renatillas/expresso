//// 2D physics world management with intelligent spatial acceleration.
////
//// This module provides the `World` type that manages a collection of rigid bodies
//// and simulates physics each step using position-based dynamics.
////
//// ## Features
////
//// - **Dual spatial acceleration**: BVH for dynamic scenes, Grid for particles
//// - **Spatial queries**: Find bodies by radius, region, or nearest neighbor
//// - **Auto-selection**: Intelligently chooses BVH or Grid based on body distribution
//// - **Deterministic**: Same inputs always produce same outputs
////
//// ## Quick Start
////
//// ```gleam
//// import expresso/world
//// import expresso/body
//// import vec/vec2
////
//// // Create a world with gravity
//// let world = world.new(gravity: vec2.Vec2(0.0, -9.8))
////
//// // Add bodies
//// let world = world
////   |> world.add_body(body.new_circle("ball", vec2.Vec2(0.0, 5.0), radius: 0.5))
////   |> world.add_body(body.new_box("platform", vec2.Vec2(0.0, 0.0), 5.0, 0.5))
////
//// // Configure solver
//// let world = world
////   |> world.with_iterations(10)
////   |> world.with_restitution(0.5)
////
//// // Step physics each frame
//// let world = world.step(world, delta_time: 0.016)
//// ```
////
//// ## Spatial Acceleration
////
//// Choose the best strategy for your use case:
////
//// - **UseBVH** - Best for dynamic scenes with moving objects (default)
//// - **UseGrid** - Best for uniform particle systems (1000s of particles)
//// - **AutoSelect** - Automatically picks BVH or Grid based on distribution
////
//// ```gleam
//// // Force BVH for dynamic game
//// let world = world.with_spatial_strategy(world, world.UseBVH)
////
//// // Force Grid for particle system
//// let world = world.with_spatial_strategy(world, world.UseGrid(cell_size: 2.0))
//// ```
////
//// ## Spatial Queries
////
//// Find bodies efficiently using spatial queries:
////
//// ```gleam
//// // Find all bodies near a point (e.g., explosion radius)
//// let nearby = world.query_radius(world, center: pos, radius: 5.0)
////
//// // Find all bodies in a rectangle (e.g., screen culling)
//// let visible = world.query_region(world, min: vec2.Vec2(-10.0, -10.0), 
////                                          max: vec2.Vec2(10.0, 10.0))
////
//// // Find nearest body (e.g., target acquisition)
//// let nearest = world.query_nearest(world, point: player_pos)
//// ```

import expresso/body.{type Body}
import expresso/collision.{type SpatialIndex, BVHIndex, GridIndex}
import expresso/internal/solver
import gleam/dict.{type Dict}
import gleam/float
import gleam/int
import gleam/list
import gleam/option.{type Option}
import quaternion
import spatial/bvh
import spatial/collider
import spatial/grid
import vec/vec3.{type Vec3}
import vec/vec3f

/// Spatial partitioning strategy
pub type SpatialStrategy {
  /// Use BVH for dynamic scenes (default)
  UseBVH
  /// Use Grid for uniform distributions (particle systems, crowds)
  UseGrid(cell_size: Float)
  /// Auto-select based on body distribution and count
  AutoSelect
}

/// Collision event types
pub type CollisionEvent(id) {
  /// A collision between two bodies started this frame
  CollisionStarted(body_a: id, body_b: id)
  /// A collision between two bodies ended this frame
  CollisionEnded(body_a: id, body_b: id)
  /// A body entered a trigger zone
  TriggerEntered(body: id, trigger: id)
  /// A body exited a trigger zone
  TriggerExited(body: id, trigger: id)
}

/// Represents a collision pair (unordered)
pub type CollisionPair(id) {
  CollisionPair(a: id, b: id)
}

/// A 3D physics world with spatial acceleration
pub opaque type World(id) {
  World(
    /// All bodies in the world (indexed by ID)
    bodies: Dict(id, Body(id)),
    /// Gravity force (units/second²)
    gravity: Vec3(Float),
    /// Number of solver iterations per step (3-5 typical)
    iterations: Int,
    /// Coefficient of restitution (bounciness: 0.0 = no bounce, 1.0 = perfectly elastic)
    restitution: Float,
    /// Number of substeps per step (2-4 typical, handles chain collisions)
    substeps: Int,
    /// Spatial partitioning strategy
    strategy: SpatialStrategy,
    /// Cached spatial index (rebuilt each step)
    spatial_index: Option(SpatialIndex(id)),
    /// Active collisions from previous frame (for event tracking)
    previous_collisions: List(CollisionPair(id)),
  )
}

/// Create a new empty physics world
///
/// ## Example
/// ```gleam
/// let world = world.new(
///   gravity: vec3.Vec3(0.0, -9.8, 0.0),  // Earth gravity pointing down
/// )
/// ```
pub fn new(gravity gravity: Vec3(Float)) -> World(id) {
  World(
    bodies: dict.new(),
    gravity: gravity,
    iterations: 4,
    // 4 iterations for stable collision resolution
    restitution: 0.0,
    // No bounce by default
    substeps: 2,
    // 2 substeps by default (handles chain collisions)
    strategy: AutoSelect,
    // Auto-select best spatial structure
    spatial_index: option.None,
    previous_collisions: [],
  )
}

/// Add a body to the world
pub fn add_body(world: World(id), body: Body(id)) -> World(id) {
  let new_bodies = dict.insert(world.bodies, body.id, body)

  // Update spatial index incrementally if it exists
  let new_spatial_index = case world.spatial_index {
    option.None -> option.None
    option.Some(BVHIndex(tree)) -> {
      let pos_3d = vec3.Vec3(body.position.x, body.position.y, 0.0)
      let updated_tree = bvh.insert(tree, pos_3d, body.id, max_leaf_size: 8)
      option.Some(BVHIndex(tree: updated_tree))
    }
    option.Some(GridIndex(grid)) -> {
      // For grid, use incremental insert as well
      let pos_3d = vec3.Vec3(body.position.x, body.position.y, 0.0)
      let updated_grid = grid.insert(grid, pos_3d, body.id)
      option.Some(GridIndex(grid: updated_grid))
    }
  }

  World(..world, bodies: new_bodies, spatial_index: new_spatial_index)
}

/// Remove a body from the world
pub fn remove_body(world: World(id), id: id) -> World(id) {
  let new_bodies = dict.delete(world.bodies, id)

  // Update spatial index incrementally if it exists
  let new_spatial_index = case world.spatial_index {
    option.None -> option.None
    option.Some(BVHIndex(tree)) -> {
      case bvh.remove(tree, fn(body_id) { body_id == id }) {
        Ok(updated_tree) -> option.Some(BVHIndex(tree: updated_tree))
        Error(_) -> option.None
        // Body wasn't in tree, clear index
      }
    }
    option.Some(GridIndex(_grid)) -> {
      // For grid, we need to know the position to remove
      // Since we're deleting from bodies dict, we can't get it
      // Fallback: invalidate the grid
      option.None
    }
  }

  World(..world, bodies: new_bodies, spatial_index: new_spatial_index)
}

/// Update a body in the world
pub fn update_body(
  world: World(id),
  id: id,
  update: fn(Body(id)) -> Body(id),
) -> World(id) {
  case dict.get(world.bodies, id) {
    Ok(body) -> {
      let updated_body = update(body)
      let new_bodies = dict.insert(world.bodies, id, updated_body)

      // Update spatial index incrementally if it exists
      let new_spatial_index = case world.spatial_index {
        option.None -> option.None
        option.Some(BVHIndex(tree)) -> {
          let new_pos =
            vec3.Vec3(updated_body.position.x, updated_body.position.y, 0.0)
          case
            bvh.update(
              tree,
              fn(body_id) { body_id == id },
              new_pos,
              id,
              max_leaf_size: 8,
            )
          {
            Ok(updated_tree) -> option.Some(BVHIndex(tree: updated_tree))
            Error(_) -> option.None
          }
        }
        option.Some(GridIndex(_)) -> {
          // For grid, invalidate (could implement incremental update later)
          option.None
        }
      }

      World(..world, bodies: new_bodies, spatial_index: new_spatial_index)
    }
    Error(_) -> world
  }
}

/// Get a body from the world
pub fn get_body(world: World(id), id: id) -> Result(Body(id), Nil) {
  dict.get(world.bodies, id)
}

/// Get all bodies in the world
pub fn bodies(world: World(id)) -> Dict(id, Body(id)) {
  world.bodies
}

/// Set the number of solver iterations
///
/// More iterations = more stable but slower.
/// 3-5 is typical, 10+ for very stable stacks.
pub fn with_iterations(world: World(id), iterations: Int) -> World(id) {
  World(..world, iterations: iterations)
}

/// Set the coefficient of restitution (bounciness)
///
/// 0.0 = perfectly inelastic (no bounce)
/// 1.0 = perfectly elastic (full bounce)
pub fn with_restitution(world: World(id), restitution: Float) -> World(id) {
  World(..world, restitution: restitution)
}

/// Set the number of substeps per physics step
///
/// Substeps divide each physics step into smaller timesteps,
/// re-detecting collisions between each substep. This fixes chain
/// collisions where forces need to propagate through multiple bodies.
///
/// More substeps = more accurate but slower.
/// Typical values: 2-3 for games, 4+ for complex chains.
///
/// ## Example
/// ```gleam
/// // Handle chains of 10+ enemies pushing each other
/// let world = world.new(gravity: vec2.Vec2(0.0, 0.0))
///   |> world.with_substeps(3)
/// ```
pub fn with_substeps(world: World(id), substeps: Int) -> World(id) {
  World(..world, substeps: substeps)
}

/// Set the spatial partitioning strategy
///
/// - `UseBVH`: Best for dynamic scenes with moving objects (default)
/// - `UseGrid(cell_size)`: Best for uniform distributions (particles, crowds)
/// - `AutoSelect`: Automatically choose based on body count
///
/// ## Example
/// ```gleam
/// // Force BVH for dynamic game
/// let world = world.with_spatial_strategy(world, UseBVH)
/// 
/// // Use grid for particle system
/// let world = world.with_spatial_strategy(world, UseGrid(cell_size: 2.0))
/// ```
pub fn with_spatial_strategy(
  world: World(id),
  strategy: SpatialStrategy,
) -> World(id) {
  World(..world, strategy: strategy, spatial_index: option.None)
}

/// Query bodies within a radius of a point
///
/// Uses spatial acceleration when available.
///
/// ## Example
/// ```gleam
/// let nearby = world.query_radius(world, center: vec3.Vec3(0.0, 0.0, 0.0), radius: 5.0)
/// ```
pub fn query_radius(
  world: World(id),
  center center: Vec3(Float),
  radius radius: Float,
) -> List(#(id, Body(id))) {
  case world.spatial_index {
    option.None -> {
      // No spatial index - brute force
      dict.to_list(world.bodies)
      |> collision.filter_by_radius(center, radius)
    }
    option.Some(BVHIndex(tree)) -> {
      bvh.query_radius(tree, center, radius)
      |> collision.extract_bodies_from_results(world.bodies)
    }
    option.Some(GridIndex(grid)) -> {
      grid.query_radius(grid, center, radius)
      |> collision.extract_bodies_from_results(world.bodies)
    }
  }
}

/// Query bodies within a rectangular region
///
/// ## Example
/// ```gleam
/// let bodies_in_region = world.query_region(
///   world,
///   min: vec3.Vec3(-10.0, -10.0, -10.0),
///   max: vec3.Vec3(10.0, 10.0, 10.0),
/// )
/// ```
pub fn query_region(
  world: World(id),
  min min: Vec3(Float),
  max max: Vec3(Float),
) -> List(#(id, Body(id))) {
  let region = collider.box(min: min, max: max)

  case world.spatial_index {
    option.None -> {
      // No spatial index - brute force
      dict.to_list(world.bodies)
      |> collision.filter_by_region(min, max)
    }
    option.Some(BVHIndex(tree)) -> {
      bvh.query(tree, region)
      |> collision.extract_bodies_from_results(world.bodies)
    }
    option.Some(GridIndex(grid)) -> {
      grid.query(grid, region)
      |> collision.extract_bodies_from_results(world.bodies)
    }
  }
}

/// Find the nearest body to a point
///
/// ## Example
/// ```gleam
/// case world.query_nearest(world, point: vec3.Vec3(5.0, 5.0, 5.0)) {
///   Some(#(id, body, distance)) -> // Use nearest body
///   None -> // No bodies in world
/// }
/// ```
pub fn query_nearest(
  world: World(id),
  point point: Vec3(Float),
) -> option.Option(#(id, Body(id), Float)) {
  let max_radius = 1000.0

  query_radius(world, center: point, radius: max_radius)
  |> collision.find_nearest(point)
}

/// Cast a ray and find the first body it hits
///
/// Returns detailed information about the hit including:
/// - The body that was hit
/// - The exact hit point
/// - The surface normal at the hit point
/// - The distance from the ray origin
///
/// Optionally filter by collision layers using the layer_mask parameter.
///
/// ## Example
/// ```gleam
/// // Shoot a laser forward
/// case world.raycast(
///   world,
///   origin: player_pos,
///   direction: vec3.Vec3(1.0, 0.0, 0.0),
///   max_distance: 100.0,
///   layer_mask: option.None,  // Hit all layers
/// ) {
///   Some(hit) -> {
///     // Hit something!
///     io.println("Hit " <> hit.body.id <> " at distance " <> float.to_string(hit.distance))
///   }
///   None -> {
///     // No hit
///   }
/// }
///
/// // Only hit enemies
/// case world.raycast(
///   world,
///   origin: player_pos,
///   direction: aim_direction,
///   max_distance: 50.0,
///   layer_mask: option.Some(body.layer_enemy),
/// ) {
///   Some(hit) -> damage_enemy(hit.body_id)
///   None -> {}
/// }
/// ```
pub fn raycast(
  world: World(id),
  origin origin: Vec3(Float),
  direction direction: Vec3(Float),
  max_distance max_distance: Float,
  layer_mask layer_mask: option.Option(Int),
) -> option.Option(collision.RaycastHit(id)) {
  collision.raycast(
    world.bodies,
    world.spatial_index,
    origin,
    direction,
    max_distance,
    layer_mask,
  )
}

/// Simulate one physics timestep using substeps
///
/// This is the main physics update function. Call it each game tick with delta_time.
/// Returns the updated world and a list of collision events that occurred.
///
/// The timestep is divided into `world.substeps` smaller steps, with collision
/// detection happening between each substep. This fixes chain collisions where
/// forces need to propagate through multiple bodies (e.g., Enemy1 → Enemy2 → Enemy3).
///
/// ## Example
/// ```gleam
/// let #(world, events) = world.step(world, delta_time: 1.0 /. 60.0)  // 60 FPS
///
/// // Handle collision events
/// list.each(events, fn(event) {
///   case event {
///     CollisionStarted(a, b) -> io.println("Collision started!")
///     TriggerEntered(body, trigger) -> io.println("Entered trigger zone!")
///     _ -> Nil
///   }
/// })
/// ```
pub fn step(
  world world: World(id),
  delta_time delta_time: Float,
) -> #(World(id), List(CollisionEvent(id))) {
  // Check if we need extra substeps for CCD
  let needs_ccd = has_fast_ccd_bodies(world, delta_time)

  // Use more substeps if CCD is needed
  let substeps = case needs_ccd {
    True -> world.substeps * 4
    // 4x more substeps for CCD
    False -> world.substeps
  }

  // Divide timestep into substeps for better collision handling
  let substep_dt = delta_time /. int.to_float(substeps)

  // Run physics multiple times with smaller timesteps
  // Only generate events on the final substep
  let #(final_world, events) =
    list.range(0, substeps - 1)
    |> list.fold(#(world, []), fn(acc, substep_index) {
      let #(current_world, _prev_events) = acc
      let is_final_substep = substep_index == substeps - 1

      let updated_world =
        current_world
        |> apply_gravity(substep_dt)
        |> integrate_velocities(substep_dt)

      detect_and_resolve_collisions(updated_world, is_final_substep)
    })

  #(final_world, events)
}

/// Check if world has fast-moving CCD bodies that need extra substeps
fn has_fast_ccd_bodies(world: World(id), delta_time: Float) -> Bool {
  // CCD threshold: if body moves more than its bounding radius per frame, use CCD
  dict.to_list(world.bodies)
  |> list.any(fn(pair) {
    let #(_id, body_val) = pair
    case body_val.use_ccd {
      False -> False
      True -> {
        let speed = vec3f.length(body_val.velocity)
        let distance_per_frame = speed *. delta_time
        let bounding_radius = body.bounding_radius(body_val)

        // Use CCD if moving more than half the bounding radius per frame
        distance_per_frame >. bounding_radius *. 0.5
      }
    }
  })
}

/// Apply gravity to all dynamic bodies
fn apply_gravity(world: World(id), delta_time: Float) -> World(id) {
  let new_bodies =
    dict.map_values(world.bodies, fn(_id, body) {
      case body.is_kinematic {
        True -> body
        False -> {
          let gravity_velocity = vec3f.scale(world.gravity, delta_time)
          body.Body(
            ..body,
            velocity: vec3f.add(body.velocity, gravity_velocity),
          )
        }
      }
    })

  World(..world, bodies: new_bodies)
}

/// Integrate velocities to update positions and rotations
fn integrate_velocities(world: World(id), delta_time: Float) -> World(id) {
  let new_bodies =
    dict.map_values(world.bodies, fn(_id, body) {
      // Calculate new position
      let displacement = vec3f.scale(body.velocity, delta_time)
      let new_position = vec3f.add(body.position, displacement)

      // Apply friction (damping)
      let friction_factor = 1.0 -. body.friction *. delta_time
      let new_velocity = vec3f.scale(body.velocity, friction_factor)

      // Integrate angular velocity into rotation
      let new_rotation =
        integrate_angular_velocity(
          body.rotation,
          body.angular_velocity,
          delta_time,
        )

      // Apply angular damping
      let angular_damping = 0.98
      // Slight damping to prevent infinite spinning
      let new_angular_velocity =
        vec3f.scale(body.angular_velocity, angular_damping)

      body.Body(
        ..body,
        position: new_position,
        velocity: new_velocity,
        rotation: new_rotation,
        angular_velocity: new_angular_velocity,
      )
    })

  World(..world, bodies: new_bodies)
}

/// Integrate angular velocity into rotation using quaternions
fn integrate_angular_velocity(
  rotation: quaternion.Quaternion,
  angular_velocity: Vec3(Float),
  delta_time: Float,
) -> quaternion.Quaternion {
  // Calculate the magnitude of angular velocity (rotation speed in rad/s)
  let speed_squared =
    angular_velocity.x
    *. angular_velocity.x
    +. angular_velocity.y
    *. angular_velocity.y
    +. angular_velocity.z
    *. angular_velocity.z

  // If angular velocity is near zero, no rotation change
  case speed_squared <. 0.000001 {
    True -> rotation
    False -> {
      // Calculate rotation angle for this timestep
      let speed = case float.square_root(speed_squared) {
        Ok(s) -> s
        Error(_) -> 0.0
      }
      let angle = speed *. delta_time

      // Get rotation axis (normalized angular velocity)
      let axis =
        vec3.Vec3(
          angular_velocity.x /. speed,
          angular_velocity.y /. speed,
          angular_velocity.z /. speed,
        )

      // Create delta quaternion from axis-angle
      let delta_rotation = quaternion.from_axis_angle(axis, angle)

      // Apply rotation: new_rotation = rotation * delta_rotation
      quaternion.multiply(rotation, delta_rotation)
      |> quaternion.normalize
    }
  }
}

/// Detect collisions and resolve them
/// Returns updated world and collision events (only generated if generate_events is True)
fn detect_and_resolve_collisions(
  world: World(id),
  generate_events: Bool,
) -> #(World(id), List(CollisionEvent(id))) {
  // Rebuild spatial index with current body positions (very fast with ID-based approach)
  let world = rebuild_spatial_index(world)

  // Detect all collisions using spatial index
  let contacts = case world.spatial_index {
    option.None -> collision.detect_collisions(world.bodies)
    option.Some(spatial_index) ->
      collision.detect_collisions_with_index(world.bodies, spatial_index)
  }

  // Resolve position constraints (separation)
  let bodies_after_position =
    solver.solve_position_constraints(world.bodies, contacts, world.iterations)

  // Resolve velocity constraints (bounce)
  let bodies_after_velocity =
    solver.solve_velocity_constraints(
      bodies_after_position,
      contacts,
      world.restitution,
    )

  // Convert contacts to collision pairs
  let #(current_regular, current_triggers) =
    contacts_to_pairs(contacts, world.bodies)
  let all_current = list.append(current_regular, current_triggers)

  // Generate collision events if requested
  let events = case generate_events {
    False -> []
    True -> {
      generate_collision_events(
        current_regular,
        current_triggers,
        world.previous_collisions,
        world.bodies,
      )
    }
  }

  // Only update previous_collisions when generating events (end of full step)
  let new_previous = case generate_events {
    True -> all_current
    False -> world.previous_collisions
  }

  // Keep the spatial index structure - just needs rebuild with new positions next frame
  // The rebuild is very fast (40x faster than old approach) with ID-based storage
  let updated_world =
    World(
      ..world,
      bodies: bodies_after_velocity,
      spatial_index: world.spatial_index,
      previous_collisions: new_previous,
    )

  #(updated_world, events)
}

/// Build or rebuild the spatial index based on current bodies
fn rebuild_spatial_index(world: World(id)) -> World(id) {
  let body_count = dict.size(world.bodies)

  case body_count {
    0 -> World(..world, spatial_index: option.None)
    _ -> {
      let strategy = case world.strategy {
        UseBVH -> UseBVH
        UseGrid(cell_size) -> UseGrid(cell_size)
        AutoSelect -> auto_select_strategy(world.bodies, body_count)
      }

      let spatial_index = case strategy {
        UseBVH -> build_bvh_index(world.bodies)
        UseGrid(cell_size) -> build_grid_index(world.bodies, cell_size)
        AutoSelect -> build_bvh_index(world.bodies)
      }

      World(..world, spatial_index: option.Some(spatial_index))
    }
  }
}

/// Auto-select the best spatial strategy based on body count and distribution
fn auto_select_strategy(
  bodies: Dict(id, Body(id)),
  body_count: Int,
) -> SpatialStrategy {
  case body_count {
    n if n < 100 -> UseBVH
    _ -> {
      let #(is_uniform, avg_radius) = collision.analyze_distribution(bodies)
      case is_uniform {
        True -> UseGrid(cell_size: avg_radius *. 4.0)
        False -> UseBVH
      }
    }
  }
}

/// Build a BVH spatial index from bodies (stores IDs for efficient updates)
fn build_bvh_index(bodies: Dict(id, Body(id))) -> SpatialIndex(id) {
  let items =
    dict.to_list(bodies)
    |> list.map(fn(pair) {
      let #(id, body) = pair
      let pos_3d = vec3.Vec3(body.position.x, body.position.y, 0.0)
      #(pos_3d, id)
    })

  let assert Ok(tree) = bvh.from_items(items, max_leaf_size: 8)
    as "Expected body count to be always nonzero"

  BVHIndex(tree: tree)
}

/// Build a Grid spatial index from bodies (stores IDs for efficient updates)
fn build_grid_index(
  bodies: Dict(id, Body(id)),
  cell_size: Float,
) -> SpatialIndex(id) {
  let bounds = collision.calculate_world_bounds(bodies)

  let grid_empty = grid.new(cell_size: cell_size, bounds: bounds)

  let grid_filled =
    dict.fold(bodies, grid_empty, fn(g, id, body) {
      let pos_3d = vec3.Vec3(body.position.x, body.position.y, 0.0)
      grid.insert(g, pos_3d, id)
    })

  GridIndex(grid: grid_filled)
}

// =============================================================================
// Collision Event Detection
// =============================================================================

/// Check if a collision pair exists in a list (order-independent)
fn contains_collision(collisions: List(CollisionPair(id)), a: id, b: id) -> Bool {
  list.any(collisions, fn(pair) {
    { pair.a == a && pair.b == b } || { pair.a == b && pair.b == a }
  })
}

/// Convert contacts to collision pairs
fn contacts_to_pairs(
  contacts: List(collision.Contact(id)),
  bodies: Dict(id, Body(id)),
) -> #(List(CollisionPair(id)), List(CollisionPair(id))) {
  // Separate into regular collisions and trigger events
  list.fold(contacts, #([], []), fn(acc, contact) {
    let #(regular, triggers) = acc

    case dict.get(bodies, contact.body_a), dict.get(bodies, contact.body_b) {
      Ok(body_a), Ok(body_b) -> {
        let pair = CollisionPair(a: contact.body_a, b: contact.body_b)
        case body_a.is_trigger || body_b.is_trigger {
          True -> #(regular, [pair, ..triggers])
          False -> #([pair, ..regular], triggers)
        }
      }
      _, _ -> acc
    }
  })
}

/// Generate collision events by comparing current and previous collisions
fn generate_collision_events(
  current_regular: List(CollisionPair(id)),
  current_triggers: List(CollisionPair(id)),
  previous: List(CollisionPair(id)),
  bodies: Dict(id, Body(id)),
) -> List(CollisionEvent(id)) {
  // Find started collisions (in current but not in previous)
  let started_events =
    list.filter_map(current_regular, fn(pair) {
      case contains_collision(previous, pair.a, pair.b) {
        True -> Error(Nil)
        False -> Ok(CollisionStarted(body_a: pair.a, body_b: pair.b))
      }
    })

  // Find ended collisions (in previous but not in current)
  let ended_events =
    list.filter_map(previous, fn(pair) {
      case contains_collision(current_regular, pair.a, pair.b) {
        True -> Error(Nil)
        False -> Ok(CollisionEnded(body_a: pair.a, body_b: pair.b))
      }
    })

  // Handle trigger events
  let trigger_events =
    list.filter_map(current_triggers, fn(pair) {
      case dict.get(bodies, pair.a), dict.get(bodies, pair.b) {
        Ok(body_a), Ok(body_b) -> {
          // Determine which is the trigger and which is the body
          case body_a.is_trigger, body_b.is_trigger {
            True, False -> {
              // a is trigger, b is body
              case contains_collision(previous, pair.a, pair.b) {
                True -> Error(Nil)
                // Already colliding
                False -> Ok(TriggerEntered(body: pair.b, trigger: pair.a))
              }
            }
            False, True -> {
              // b is trigger, a is body
              case contains_collision(previous, pair.a, pair.b) {
                True -> Error(Nil)
                False -> Ok(TriggerEntered(body: pair.a, trigger: pair.b))
              }
            }
            _, _ -> Error(Nil)
            // Both triggers or neither (shouldn't happen)
          }
        }
        _, _ -> Error(Nil)
      }
    })

  // Trigger exit events (triggers in previous but not in current)
  let trigger_exit_events =
    list.filter_map(previous, fn(pair) {
      case
        dict.get(bodies, pair.a),
        dict.get(bodies, pair.b),
        contains_collision(current_triggers, pair.a, pair.b)
      {
        Ok(body_a), Ok(body_b), False -> {
          // Was colliding, now not
          case body_a.is_trigger, body_b.is_trigger {
            True, False -> Ok(TriggerExited(body: pair.b, trigger: pair.a))
            False, True -> Ok(TriggerExited(body: pair.a, trigger: pair.b))
            _, _ -> Error(Nil)
          }
        }
        _, _, _ -> Error(Nil)
      }
    })

  list.flatten([
    started_events,
    ended_events,
    trigger_events,
    trigger_exit_events,
  ])
}
