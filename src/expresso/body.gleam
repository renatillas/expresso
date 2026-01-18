//// Rigid body physics with flexible collider shapes.
////
//// This module provides the `Body` type and functions for creating and managing
//// rigid bodies in a 2D physics simulation.
////
//// ## Collider Shapes
////
//// Bodies can use any of the collider shapes from the `spatial` library:
////
//// - **Sphere** - Fast, perfect for projectiles and particles
//// - **Box** - Axis-aligned bounding boxes for walls, platforms, crates
//// - **Capsule** - Ideal for character controllers (no corner catching!)
//// - **Cylinder** - For cylindrical objects like pillars or barrels
////
//// ## Body Types
////
//// - **Dynamic** - Affected by forces and collisions (enemies, objects, particles)
//// - **Kinematic** - Controlled externally, not affected by forces (player, moving platforms)
////
//// ## Quick Start
////
//// ```gleam
//// import expresso/body
//// import vec/vec2
////
//// // Create a dynamic circle (common case)
//// let bullet = body.new_circle("bullet", vec2.Vec2(0.0, 0.0), radius: 0.25)
////   |> body.with_velocity(vec2.Vec2(10.0, 0.0))
////
//// // Create a kinematic character with capsule collider
//// let player = body.new_capsule("player", vec2.Vec2(0.0, 0.0), height: 2.0, radius: 0.5)
////   |> body.kinematic()
////
//// // Create a static wall with box collider
//// let wall = body.new_box("wall", vec2.Vec2(5.0, 0.0), half_width: 5.0, half_height: 1.0)
////   |> body.with_mass(0.0)  // Infinite mass = static
//// ```
////
//// ## Advanced Usage
////
//// For full control over the collider, use `body.new()` with a custom collider:
////
//// ```gleam
//// import spatial/collider
//// import vec/vec3
////
//// let custom_collider = collider.cylinder(
////   center: vec3.Vec3(0.0, 0.0, 0.0),
////   radius: 1.0,
////   height: 3.0,
//// )
//// let tower = body.new("tower", vec2.Vec2(5.0, 0.0), custom_collider)
//// ```

import gleam/float
import gleam/int
import gleam/list
import quaternion.{type Quaternion}
import spatial/collider.{type Collider}
import vec/vec3.{type Vec3}
import vec/vec3f

/// Collision layer (bitmask for flexible layer management)
///
/// Use powers of 2 for layers: 1, 2, 4, 8, 16, etc.
/// This allows combining multiple layers with bitwise OR.
pub type CollisionLayer =
  Int

pub const layer_0: CollisionLayer = 1

pub const layer_1: CollisionLayer = 2

pub const layer_2: CollisionLayer = 4

pub const layer_3: CollisionLayer = 8

pub const layer_4: CollisionLayer = 16

pub const layer_5: CollisionLayer = 32

pub const layer_all: CollisionLayer = 0xFFFFFFFF

/// A 3D rigid body with configurable collider
pub type Body(id) {
  Body(
    /// Unique identifier for this body
    id: id,
    /// Current position in 3D space
    position: Vec3(Float),
    /// Current velocity (units per second)
    velocity: Vec3(Float),
    /// Current rotation (quaternion)
    rotation: Quaternion,
    /// Current angular velocity (radians per second, axis-angle form)
    angular_velocity: Vec3(Float),
    /// Collision shape (Sphere, Box, Capsule, or Cylinder)
    collider: Collider,
    /// Mass (0.0 = infinite mass)
    mass: Float,
    /// If true, body is not affected by physics (player-controlled)
    is_kinematic: Bool,
    /// Velocity damping (0.0 = no damping, 1.0 = full damping)
    friction: Float,
    /// Static friction coefficient (prevents sliding, typical: 0.3-0.6)
    static_friction: Float,
    /// Dynamic friction coefficient (opposes sliding, typical: 0.2-0.4)
    dynamic_friction: Float,
    /// Enable continuous collision detection for fast-moving objects
    use_ccd: Bool,
    /// Collision layer this body belongs to (bitmask)
    layer: CollisionLayer,
    /// Layers this body can collide with (bitmask)
    collision_mask: CollisionLayer,
    /// If true, body is a trigger (detects collisions but doesn't physically interact)
    is_trigger: Bool,
  )
}

/// Create a new dynamic body with a sphere collider
///
/// This is a convenience constructor for spherical bodies.
///
/// ## Example
/// ```gleam
/// let enemy = body.new_sphere(
///   id: "enemy_1",
///   position: vec3.Vec3(10.0, 5.0, 0.0),
///   radius: 0.5,
/// )
/// ```
pub fn new_sphere(
  id id: id,
  position position: Vec3(Float),
  radius radius: Float,
) -> Body(id) {
  Body(
    id: id,
    position: position,
    velocity: vec3.Vec3(0.0, 0.0, 0.0),
    rotation: quaternion.identity,
    angular_velocity: vec3.Vec3(0.0, 0.0, 0.0),
    collider: collider.sphere(center: vec3.Vec3(0.0, 0.0, 0.0), radius: radius),
    mass: 1.0,
    is_kinematic: False,
    friction: 0.1,
    static_friction: 0.5,
    dynamic_friction: 0.3,
    use_ccd: False,
    layer: layer_0,
    collision_mask: layer_all,
    is_trigger: False,
  )
}

/// Create a new dynamic body with a box collider
///
/// ## Example
/// ```gleam
/// let wall = body.new_box(
///   id: "wall_1",
///   position: vec3.Vec3(10.0, 5.0, 0.0),
///   half_extents: vec3.Vec3(5.0, 0.5, 0.5),
/// )
/// ```
pub fn new_box(
  id id: id,
  position position: Vec3(Float),
  half_extents half_extents: Vec3(Float),
) -> Body(id) {
  Body(
    id: id,
    position: position,
    velocity: vec3.Vec3(0.0, 0.0, 0.0),
    rotation: quaternion.identity,
    angular_velocity: vec3.Vec3(0.0, 0.0, 0.0),
    collider: collider.box_from_center(
      center: position,
      half_extents: half_extents,
    ),
    mass: 1.0,
    is_kinematic: False,
    friction: 0.1,
    static_friction: 0.5,
    dynamic_friction: 0.3,
    use_ccd: False,
    layer: layer_0,
    collision_mask: layer_all,
    is_trigger: False,
  )
}

/// Create a new dynamic body with a capsule collider
///
/// Perfect for character controllers!
///
/// ## Example
/// ```gleam
/// let player = body.new_capsule(
///   id: "player",
///   position: vec3.Vec3(0.0, 0.0, 0.0),
///   height: 2.0,
///   radius: 0.5,
/// )
/// ```
pub fn new_capsule(
  id id: id,
  position position: Vec3(Float),
  height height: Float,
  radius radius: Float,
) -> Body(id) {
  let half_height = height /. 2.0
  let start = vec3.Vec3(position.x, position.y, position.z -. half_height)
  let end = vec3.Vec3(position.x, position.y, position.z +. half_height)
  Body(
    id: id,
    position: position,
    velocity: vec3.Vec3(0.0, 0.0, 0.0),
    rotation: quaternion.identity,
    angular_velocity: vec3.Vec3(0.0, 0.0, 0.0),
    collider: collider.capsule(start: start, end: end, radius: radius),
    mass: 1.0,
    is_kinematic: False,
    friction: 0.1,
    static_friction: 0.5,
    dynamic_friction: 0.3,
    use_ccd: False,
    layer: layer_0,
    collision_mask: layer_all,
    is_trigger: False,
  )
}

/// Create a new body with a custom collider
///
/// Use this for full control over the collider shape.
///
/// ## Example
/// ```gleam
/// let custom_collider = collider.sphere(
///   center: vec3.Vec3(0.0, 0.0, 0.0),
///   radius: 2.0,
/// )
/// let body = body.new(
///   id: "custom",
///   position: vec3.Vec3(0.0, 0.0, 0.0),
///   collider: custom_collider,
/// )
/// ```
pub fn new(
  id id: id,
  position position: Vec3(Float),
  collider collider: Collider,
) -> Body(id) {
  Body(
    id: id,
    position: position,
    velocity: vec3.Vec3(0.0, 0.0, 0.0),
    rotation: quaternion.identity,
    angular_velocity: vec3.Vec3(0.0, 0.0, 0.0),
    collider: collider,
    mass: 1.0,
    is_kinematic: False,
    friction: 0.1,
    static_friction: 0.5,
    dynamic_friction: 0.3,
    use_ccd: False,
    layer: layer_0,
    collision_mask: layer_all,
    is_trigger: False,
  )
}

/// Make a body kinematic (not affected by physics)
///
/// Kinematic bodies can push dynamic bodies but are not pushed themselves.
/// Useful for player-controlled entities.
pub fn kinematic(body: Body(id)) -> Body(id) {
  Body(..body, is_kinematic: True, mass: 0.0)
}

/// Make a body dynamic (affected by physics)
pub fn dynamic(body: Body(id)) -> Body(id) {
  Body(..body, is_kinematic: False, mass: 1.0)
}

/// Set the mass of a body
pub fn with_mass(body: Body(id), mass: Float) -> Body(id) {
  Body(..body, mass: mass)
}

/// Set the velocity damping of a body (0.0 = no damping, 1.0 = full damping)
pub fn with_friction(body: Body(id), friction: Float) -> Body(id) {
  Body(..body, friction: friction)
}

/// Set the static friction coefficient (prevents sliding, typical: 0.3-0.6)
///
/// Static friction must be overcome to start sliding.
/// Higher values make objects harder to push.
pub fn with_static_friction(body: Body(id), friction: Float) -> Body(id) {
  Body(..body, static_friction: friction)
}

/// Set the dynamic friction coefficient (opposes sliding, typical: 0.2-0.4)
///
/// Dynamic friction opposes motion while sliding.
/// Should usually be less than static friction.
pub fn with_dynamic_friction(body: Body(id), friction: Float) -> Body(id) {
  Body(..body, dynamic_friction: friction)
}

/// Enable continuous collision detection for fast-moving objects
///
/// CCD prevents fast objects from tunneling through thin obstacles.
/// Use for bullets, fast projectiles, or critical objects.
/// Has a small performance cost.
pub fn with_ccd(body: Body(id), enabled: Bool) -> Body(id) {
  Body(..body, use_ccd: enabled)
}

/// Set the velocity of a body
pub fn with_velocity(body: Body(id), velocity: Vec3(Float)) -> Body(id) {
  Body(..body, velocity: velocity)
}

/// Set the angular velocity of a body (axis-angle form, radians per second)
pub fn with_angular_velocity(
  body: Body(id),
  angular_velocity: Vec3(Float),
) -> Body(id) {
  Body(..body, angular_velocity: angular_velocity)
}

/// Set the rotation of a body
pub fn with_rotation(body: Body(id), rotation: Quaternion) -> Body(id) {
  Body(..body, rotation: rotation)
}

/// Set the collision layer of a body
///
/// The layer determines which "group" this body belongs to.
/// Use the predefined constants or custom powers of 2.
///
/// ## Example
/// ```gleam
/// let bullet = body.new_circle("bullet", vec2.Vec2(0.0, 0.0), 0.2)
///   |> body.with_layer(body.layer_1)
/// ```
pub fn with_layer(body: Body(id), layer: CollisionLayer) -> Body(id) {
  Body(..body, layer: layer)
}

/// Set the collision mask of a body
///
/// The collision mask determines which layers this body can collide with.
/// Use bitwise OR to combine multiple layers.
///
/// ## Example
/// ```gleam
/// // Player bullet only collides with enemies and environment
/// let bullet = body.new_circle("bullet", vec2.Vec2(0.0, 0.0), 0.2)
///   |> body.with_layer(body.layer_1)
///   |> body.with_collision_mask(combine_layers([
///        body.layer_2,
///        body.layer_3,
///      ]))
/// ```
pub fn with_collision_mask(body: Body(id), mask: CollisionLayer) -> Body(id) {
  Body(..body, collision_mask: mask)
}

/// Combine multiple collision layers into a single mask using bitwise OR
///
/// ## Example
/// ```gleam
/// let mask = combine_layers([layer_2, layer_3])
/// ```
pub fn combine_layers(layers: List(CollisionLayer)) -> CollisionLayer {
  list.fold(layers, 0, fn(acc, layer) { int.bitwise_or(acc, layer) })
}

/// Check if two bodies should collide based on their layers
///
/// Bodies collide if:
/// - Body A's layer is in Body B's collision mask, AND
/// - Body B's layer is in Body A's collision mask
pub fn should_collide(body_a: Body(id), body_b: Body(id)) -> Bool {
  let a_in_b_mask = int.bitwise_and(body_a.layer, body_b.collision_mask) != 0
  let b_in_a_mask = int.bitwise_and(body_b.layer, body_a.collision_mask) != 0
  a_in_b_mask && b_in_a_mask
}

/// Make a body a trigger (sensor)
///
/// Triggers detect collisions but don't physically interact.
/// Perfect for damage zones, checkpoints, pickups, etc.
///
/// ## Example
/// ```gleam
/// let checkpoint = body.new_circle("checkpoint", vec2.Vec2(10.0, 0.0), 1.0)
///   |> body.trigger()
/// ```
pub fn trigger(body: Body(id)) -> Body(id) {
  Body(..body, is_trigger: True, mass: 0.0)
}

/// Get the inverse mass (used in physics calculations)
///
/// Returns 0.0 for kinematic bodies or bodies with 0 mass.
pub fn inverse_mass(body: Body(id)) -> Float {
  case body.is_kinematic || body.mass == 0.0 {
    True -> 0.0
    False -> 1.0 /. body.mass
  }
}

/// Get the collider in world space (updated with body position and rotation)
///
/// This is useful for collision detection.
pub fn world_collider(body: Body(id)) -> Collider {
  // Translate and rotate collider to world space
  let world_pos = body.position
  let rotation = body.rotation

  // Apply rotation and translation to collider
  case body.collider {
    // Sphere: Rotation doesn't affect spheres (rotationally symmetric)
    collider.Sphere(_, radius) ->
      collider.sphere(center: world_pos, radius: radius)

    // Box: Rotate the half-extents and compute conservative AABB
    // Note: This creates an axis-aligned box that contains the rotated box
    collider.Box(_min, _max) -> {
      let size = collider.size(body.collider)
      let half_extents = vec3.Vec3(size.x /. 2.0, size.y /. 2.0, size.z /. 2.0)

      // Rotate the 8 corners of the box and find the new AABB
      let corners = [
        vec3.Vec3(half_extents.x, half_extents.y, half_extents.z),
        vec3.Vec3(half_extents.x, half_extents.y, 0.0 -. half_extents.z),
        vec3.Vec3(half_extents.x, 0.0 -. half_extents.y, half_extents.z),
        vec3.Vec3(half_extents.x, 0.0 -. half_extents.y, 0.0 -. half_extents.z),
        vec3.Vec3(0.0 -. half_extents.x, half_extents.y, half_extents.z),
        vec3.Vec3(0.0 -. half_extents.x, half_extents.y, 0.0 -. half_extents.z),
        vec3.Vec3(0.0 -. half_extents.x, 0.0 -. half_extents.y, half_extents.z),
        vec3.Vec3(
          0.0 -. half_extents.x,
          0.0 -. half_extents.y,
          0.0 -. half_extents.z,
        ),
      ]

      // Rotate each corner and find min/max
      let rotated_corners =
        list.map(corners, fn(corner) { quaternion.rotate(rotation, corner) })

      // Find the new AABB extents
      let new_half_extents = compute_aabb_extents(rotated_corners)
      collider.box_from_center(
        center: world_pos,
        half_extents: new_half_extents,
      )
    }

    // Capsule: Rotate the axis (start and end points) around the center
    collider.Capsule(start, end, radius) -> {
      // Get the capsule axis in local space (centered at origin)
      let half_length_x = { end.x -. start.x } /. 2.0
      let half_length_y = { end.y -. start.y } /. 2.0
      let half_length_z = { end.z -. start.z } /. 2.0

      let local_start =
        vec3.Vec3(
          0.0 -. half_length_x,
          0.0 -. half_length_y,
          0.0 -. half_length_z,
        )
      let local_end = vec3.Vec3(half_length_x, half_length_y, half_length_z)

      // Rotate the endpoints
      let rotated_start = quaternion.rotate(rotation, local_start)
      let rotated_end = quaternion.rotate(rotation, local_end)

      // Translate to world position
      let world_start = vec3f.add(world_pos, rotated_start)
      let world_end = vec3f.add(world_pos, rotated_end)

      collider.capsule(start: world_start, end: world_end, radius: radius)
    }

    // Cylinder: Similar to capsule, rotate the axis
    // Note: Cylinders are defined by center, radius, and height (axis along Y)
    collider.Cylinder(_, radius, height) -> {
      // Default cylinder axis is along Y-axis
      let half_height = height /. 2.0
      let local_bottom = vec3.Vec3(0.0, 0.0 -. half_height, 0.0)
      let local_top = vec3.Vec3(0.0, half_height, 0.0)

      // Rotate the endpoints
      let rotated_bottom = quaternion.rotate(rotation, local_bottom)
      let rotated_top = quaternion.rotate(rotation, local_top)

      // For now, use capsule representation for rotated cylinders
      // This is conservative but correct for collision detection
      let world_start = vec3f.add(world_pos, rotated_bottom)
      let world_end = vec3f.add(world_pos, rotated_top)

      collider.capsule(start: world_start, end: world_end, radius: radius)
    }
  }
}

/// Compute AABB half-extents from a list of points
fn compute_aabb_extents(points: List(Vec3(Float))) -> Vec3(Float) {
  case points {
    [] -> vec3.Vec3(0.0, 0.0, 0.0)
    [first, ..rest] -> {
      // Find min and max for each axis
      let #(min_x, max_x, min_y, max_y, min_z, max_z) =
        list.fold(
          rest,
          #(first.x, first.x, first.y, first.y, first.z, first.z),
          fn(acc, point) {
            let #(min_x, max_x, min_y, max_y, min_z, max_z) = acc
            #(
              float.min(min_x, point.x),
              float.max(max_x, point.x),
              float.min(min_y, point.y),
              float.max(max_y, point.y),
              float.min(min_z, point.z),
              float.max(max_z, point.z),
            )
          },
        )

      // Return half-extents
      vec3.Vec3(
        { max_x -. min_x } /. 2.0,
        { max_y -. min_y } /. 2.0,
        { max_z -. min_z } /. 2.0,
      )
    }
  }
}

/// Get the bounding radius of the collider (for spatial queries)
///
/// This returns the radius of the smallest sphere that contains the collider.
pub fn bounding_radius(body: Body(id)) -> Float {
  case body.collider {
    collider.Sphere(_, radius) -> radius
    collider.Box(_min, _max) -> {
      let size = collider.size(body.collider)
      let max_extent = case size.x >. size.y {
        True -> size.x
        False -> size.y
      }
      max_extent /. 2.0
    }
    collider.Capsule(start, end, radius) -> {
      let length_vec =
        vec3.Vec3(end.x -. start.x, end.y -. start.y, end.z -. start.z)
      let length_sq =
        length_vec.x
        *. length_vec.x
        +. length_vec.y
        *. length_vec.y
        +. length_vec.z
        *. length_vec.z
      let length = case length_sq {
        0.0 -> 0.0
        _ -> {
          case length_sq |> float.square_root {
            Ok(l) -> l
            Error(_) -> 0.0
          }
        }
      }
      radius +. length /. 2.0
    }
    collider.Cylinder(_, radius, height) -> {
      let max_dim = case radius >. height /. 2.0 {
        True -> radius
        False -> height /. 2.0
      }
      max_dim
    }
  }
}
