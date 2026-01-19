//// Collision detection with spatial acceleration.
////
//// This module provides efficient collision detection using spatial partitioning
//// data structures from the `spatial` library.
////
//// ## Features
////
//// - **Multi-shape support**: Sphere, Box, Capsule, and Cylinder colliders
//// - **Spatial acceleration**: BVH (O(n log n)) and Grid (O(c + k))
//// - **Adaptive selection**: Auto-choose BVH or Grid based on distribution
//// - **Accurate detection**: Uses `spatial` library's collision algorithms
////
//// ## Collision Detection Strategies
////
//// ### Brute Force (< 10 bodies)
//// - Complexity: O(n²)
//// - Used automatically for small scenes
//// - No spatial index overhead
////
//// ### BVH (10-1000+ bodies, dynamic)
//// - Complexity: O(n log n + k) where k = collision pairs
//// - Best for dynamic scenes with moving objects
//// - Handles non-uniform distributions well
////
//// ### Grid (100-1000+ bodies, uniform)
//// - Complexity: O(c + k) where c = cells checked, k = collision pairs
//// - Best for particle systems and uniformly distributed bodies
//// - Excellent for 1000s of particles
////
//// ## Usage
////
//// Most users won't call this module directly - the `world` module handles
//// collision detection automatically. However, advanced users can use these
//// functions for custom collision detection:
////
//// ```gleam
//// import expresso/collision
//// import gleam/dict
////
//// // Simple collision detection (builds BVH internally)
//// let contacts = collision.detect_collisions(bodies)
////
//// // With pre-built BVH (more efficient if reusing)
//// let bvh = collision.build_bvh(bodies)
//// let contacts = collision.detect_collisions_with_bvh(bodies, option.Some(bvh))
//// ```

import expresso/body.{type Body}
import gleam/dict.{type Dict}
import gleam/float
import gleam/int
import gleam/list
import gleam/option
import spatial/bvh.{type BVH}
import spatial/collider.{type Collider}
import spatial/grid.{type Grid}
import vec/vec3.{type Vec3}
import vec/vec3f

/// Internal spatial index type (stores IDs for efficient updates)
pub type SpatialIndex(id) {
  BVHIndex(tree: BVH(id))
  GridIndex(grid: Grid(id))
}

/// Contact information for a collision between two bodies
pub type Contact(id) {
  Contact(
    /// First body ID
    body_a: id,
    /// Second body ID
    body_b: id,
    /// Contact normal (points from A to B)
    normal: Vec3(Float),
    /// Penetration depth (how much they overlap)
    penetration: Float,
    /// Contact point in world space
    point: Vec3(Float),
  )
}

/// Result of a raycast
pub type RaycastHit(id) {
  RaycastHit(
    /// Body that was hit
    body_id: id,
    /// Body that was hit
    body: Body(id),
    /// Point where the ray hit the body
    point: Vec3(Float),
    /// Normal at the hit point
    normal: Vec3(Float),
    /// Distance from ray origin to hit point
    distance: Float,
  )
}

/// Detect all collisions between bodies
///
/// Returns a list of contacts for all overlapping pairs.
/// 
/// **Performance:**
/// - For best performance, use `detect_collisions_with_index()` with a pre-built spatial index
/// - This function uses brute force or builds a BVH internally
pub fn detect_collisions(bodies: Dict(id, Body(id))) -> List(Contact(id)) {
  detect_collisions_with_bvh(bodies, option.None)
}

/// Detect collisions using an optional pre-built BVH for acceleration
///
/// This is the recommended API for performance-critical code.
pub fn detect_collisions_with_bvh(
  bodies: Dict(id, Body(id)),
  bvh_option: option.Option(bvh.BVH(#(id, Body(id)))),
) -> List(Contact(id)) {
  let body_list = dict.to_list(bodies)

  case bvh_option {
    // BVH provided - use it
    option.Some(bvh_tree) ->
      detect_collisions_with_bvh_internal(body_list, bvh_tree)

    // No BVH - decide based on body count
    option.None ->
      case list.length(body_list) < 10 {
        True -> detect_collisions_brute_force(body_list)
        False -> {
          // Build BVH and use it
          let bvh_tree = build_bvh_internal(body_list)
          case bvh_tree {
            Error(Nil) -> []
            Ok(tree) -> detect_collisions_with_bvh_internal(body_list, tree)
          }
        }
      }
  }
}

/// Build a BVH spatial index from bodies
pub fn build_bvh(
  bodies: Dict(id, Body(id)),
) -> Result(bvh.BVH(#(id, Body(id))), Nil) {
  build_bvh_internal(dict.to_list(bodies))
}

/// Internal: Build BVH from body list
fn build_bvh_internal(
  body_list: List(#(id, Body(id))),
) -> Result(bvh.BVH(#(id, Body(id))), Nil) {
  let bvh_items =
    list.map(body_list, fn(pair) {
      let #(id, body) = pair
      let pos_3d = vec3.Vec3(body.position.x, body.position.y, 0.0)
      #(pos_3d, #(id, body))
    })

  bvh.from_items(bvh_items, max_leaf_size: 8)
}

/// Brute force collision detection for small body counts
fn detect_collisions_brute_force(
  body_list: List(#(id, Body(id))),
) -> List(Contact(id)) {
  list.flat_map(body_list, fn(pair_a) {
    let #(id_a, body_a) = pair_a

    list.filter_map(body_list, fn(pair_b) {
      let #(id_b, body_b) = pair_b

      // Only check each pair once
      case id_a == id_b {
        False -> check_collision(body_a, body_b)
        _ -> Error(Nil)
      }
    })
  })
}

/// BVH-accelerated collision detection
fn detect_collisions_with_bvh_internal(
  body_list: List(#(id, Body(id))),
  bvh_tree: bvh.BVH(#(id, Body(id))),
) -> List(Contact(id)) {
  // Calculate max radius once, not per body (O(n) instead of O(n²))
  let max_radius = calculate_max_radius(body_list)

  list.flat_map(body_list, fn(pair_a) {
    let #(id_a, body_a) = pair_a

    // Query radius = bounding radius + max possible other body radius
    let query_radius = body.bounding_radius(body_a) +. max_radius
    let pos_3d = vec3.Vec3(body_a.position.x, body_a.position.y, 0.0)

    // Get nearby candidates from BVH
    let nearby = bvh.query_radius(bvh_tree, pos_3d, query_radius)

    // Narrow-phase: check collision with each candidate
    list.filter_map(nearby, fn(candidate) {
      let #(_pos_3d, #(id_b, body_b)) = candidate

      // Skip self-collision
      case id_a == id_b {
        True -> Error(Nil)
        False ->
          // Only check each pair once
          case id_a == id_b {
            False -> check_collision(body_a, body_b)
            _ -> Error(Nil)
          }
      }
    })
  })
}

// =============================================================================
// Spatial Index Integration
// =============================================================================

/// Detect collisions using a spatial index (BVH or Grid)
pub fn detect_collisions_with_index(
  bodies: Dict(id, Body(id)),
  spatial_index: SpatialIndex(id),
) -> List(Contact(id)) {
  let body_list = dict.to_list(bodies)

  case spatial_index {
    BVHIndex(tree) -> detect_collisions_with_bvh_new(bodies, body_list, tree)
    GridIndex(grid) -> detect_collisions_with_grid_new(bodies, body_list, grid)
  }
}

/// Detect collisions using BVH (optimized version with ID-based storage)
fn detect_collisions_with_bvh_new(
  bodies: Dict(id, Body(id)),
  body_list: List(#(id, Body(id))),
  bvh_tree: BVH(id),
) -> List(Contact(id)) {
  // Calculate max radius once, not per body (O(n) instead of O(n²))
  let max_radius = calculate_max_radius(body_list)

  list.flat_map(body_list, fn(pair_a) {
    let #(id_a, body_a) = pair_a

    let query_radius = body.bounding_radius(body_a) +. max_radius
    let pos_3d = vec3.Vec3(body_a.position.x, body_a.position.y, 0.0)

    let nearby = bvh.query_radius(bvh_tree, pos_3d, query_radius)

    list.filter_map(nearby, fn(candidate) {
      let #(_pos_3d, id_b) = candidate

      case id_a == id_b {
        True -> Error(Nil)
        False -> {
          // Look up body_b from dict
          case dict.get(bodies, id_b) {
            Ok(body_b) -> check_collision(body_a, body_b)
            Error(_) -> Error(Nil)
          }
        }
      }
    })
  })
}

/// Detect collisions using a Grid spatial index (with ID-based storage)
fn detect_collisions_with_grid_new(
  bodies: Dict(id, Body(id)),
  body_list: List(#(id, Body(id))),
  grid_index: Grid(id),
) -> List(Contact(id)) {
  // Calculate max radius once, not per body (O(n) instead of O(n²))
  let max_radius = calculate_max_radius(body_list)

  list.flat_map(body_list, fn(pair_a) {
    let #(id_a, body_a) = pair_a

    let query_radius = body.bounding_radius(body_a) +. max_radius
    let pos_3d = vec3.Vec3(body_a.position.x, body_a.position.y, 0.0)

    let nearby = grid.query_radius(grid_index, pos_3d, query_radius)

    list.filter_map(nearby, fn(candidate) {
      let #(_pos_3d, id_b) = candidate

      case id_a == id_b {
        True -> Error(Nil)
        False -> {
          // Look up body_b from dict
          case dict.get(bodies, id_b) {
            Ok(body_b) -> check_collision(body_a, body_b)
            Error(_) -> Error(Nil)
          }
        }
      }
    })
  })
}

/// Check if two bodies are colliding using their colliders
///
/// Uses the spatial library's collider.intersects() for accurate collision detection.
/// Respects collision layers - bodies only collide if their layers match.
fn check_collision(
  body_a: Body(id),
  body_b: Body(id),
) -> Result(Contact(id), Nil) {
  // Check collision layers first (fast early exit)
  case body.should_collide(body_a, body_b) {
    False -> Error(Nil)
    True -> {
      let collider_a = body.world_collider(body_a)
      let collider_b = body.world_collider(body_b)

      case collider.intersects(collider_a, collider_b) {
        False -> Error(Nil)
        True -> {
          // Calculate contact details
          let center_a = collider.center(collider_a)
          let center_b = collider.center(collider_b)

          let delta = vec3f.subtract(center_b, center_a)
          let distance_3d = vec3f.length(delta)

          // Calculate penetration depth (approximate)
          let size_a = collider.size(collider_a)
          let size_b = collider.size(collider_b)
          let radius_a = float.max(size_a.x, size_a.y) /. 2.0
          let radius_b = float.max(size_b.x, size_b.y) /. 2.0
          let penetration = radius_a +. radius_b -. distance_3d

          // Normalize delta to get contact normal (3D)
          let normal = case distance_3d >. 0.0001 {
            True -> vec3f.scale(delta, 1.0 /. distance_3d)
            False -> vec3.Vec3(1.0, 0.0, 0.0)
          }

          // Contact point is halfway between surfaces
          let contact_point = vec3f.add(center_a, vec3f.scale(delta, 0.5))

          Ok(Contact(
            body_a: body_a.id,
            body_b: body_b.id,
            normal: normal,
            penetration: penetration,
            point: contact_point,
          ))
        }
      }
    }
  }
}

/// Calculate the maximum bounding radius among all bodies
fn calculate_max_radius(body_list: List(#(id, Body(id)))) -> Float {
  list.fold(body_list, 0.0, fn(max_r, pair) {
    let #(_id, body) = pair
    float.max(max_r, body.bounding_radius(body))
  })
}

// =============================================================================
// Helper Functions for World Queries
// =============================================================================

/// Convert bodies to BVH items
pub fn bodies_to_bvh_items(
  body_list: List(#(String, Body(id))),
) -> List(#(Vec3(Float), Body(id))) {
  list.map(body_list, fn(pair) {
    let #(_id, body) = pair
    let pos_3d = vec3.Vec3(body.position.x, body.position.y, 0.0)
    #(pos_3d, body)
  })
}

/// Extract body IDs from query results and lookup bodies from dict
pub fn extract_bodies_from_results(
  results: List(#(Vec3(Float), id)),
  bodies: Dict(id, Body(id)),
) -> List(#(id, Body(id))) {
  list.filter_map(results, fn(result) {
    let #(_pos, id) = result
    case dict.get(bodies, id) {
      Ok(body) -> Ok(#(id, body))
      Error(_) -> Error(Nil)
    }
  })
}

/// Filter bodies by radius (brute force)
pub fn filter_by_radius(
  body_list: List(#(id, Body(id))),
  center: Vec3(Float),
  radius: Float,
) -> List(#(id, Body(id))) {
  list.filter(body_list, fn(pair) {
    let #(_id, body) = pair
    let distance = vec3f.distance(center, body.position)
    distance <=. radius +. body.bounding_radius(body)
  })
}

/// Filter bodies by rectangular region (brute force)
pub fn filter_by_region(
  body_list: List(#(id, Body(id))),
  min: Vec3(Float),
  max: Vec3(Float),
) -> List(#(id, Body(id))) {
  list.filter(body_list, fn(pair) {
    let #(_id, body) = pair
    body.position.x >=. min.x
    && body.position.x <=. max.x
    && body.position.y >=. min.y
    && body.position.y <=. max.y
    && body.position.z >=. min.z
    && body.position.z <=. max.z
  })
}

/// Find the nearest body to a point
pub fn find_nearest(
  candidates: List(#(id, Body(id))),
  point: Vec3(Float),
) -> option.Option(#(id, Body(id), Float)) {
  list.fold(candidates, option.None, fn(best, pair) {
    let #(id, body) = pair
    let distance = vec3f.distance(point, body.position)

    case best {
      option.None -> option.Some(#(id, body, distance))
      option.Some(#(_best_id, _best_body, best_dist)) ->
        case distance <. best_dist {
          True -> option.Some(#(id, body, distance))
          False -> best
        }
    }
  })
}

/// Analyze body distribution to determine if uniform
///
/// Returns #(is_uniform, average_bounding_radius)
/// Optimized: Single pass to compute count, total_radius, and positions
pub fn analyze_distribution(bodies: Dict(id, Body(id))) -> #(Bool, Float) {
  // Single pass: compute count, total_radius, and positions simultaneously
  let #(count, total_radius, positions) =
    dict.fold(bodies, #(0, 0.0, []), fn(acc, _id, body) {
      let #(n, sum_radius, pos_list) = acc
      #(n + 1, sum_radius +. body.bounding_radius(body), [
        body.position,
        ..pos_list
      ])
    })

  case count {
    0 -> #(False, 1.0)
    _ -> {
      let avg_radius = total_radius /. int.to_float(count)

      // Calculate variance in spacing
      let variance = calculate_position_variance(positions)

      // If variance is low relative to world size, distribution is uniform
      let is_uniform = variance <. 100.0

      #(is_uniform, avg_radius)
    }
  }
}

/// Calculate variance in positions (simplified)
/// Optimized: 2 passes instead of 3 (combined count+sum in first pass)
fn calculate_position_variance(positions: List(Vec3(Float))) -> Float {
  // First pass: calculate count and sum simultaneously
  let #(n, sum) =
    list.fold(positions, #(0, vec3.Vec3(0.0, 0.0, 0.0)), fn(acc, pos) {
      let #(count, sum_vec) = acc
      #(count + 1, vec3f.add(sum_vec, pos))
    })

  case n {
    0 -> 0.0
    _ -> {
      let centroid = vec3f.scale(sum, 1.0 /. int.to_float(n))

      // Second pass: calculate average squared distance from centroid
      let total_sq_dist =
        list.fold(positions, 0.0, fn(acc, pos) {
          let dist = vec3f.distance(centroid, pos)
          acc +. { dist *. dist }
        })

      total_sq_dist /. int.to_float(n)
    }
  }
}

/// Calculate world bounds for Grid creation
/// Optimized: Single pass with dict.fold instead of dict.to_list + list.fold
pub fn calculate_world_bounds(bodies: Dict(id, Body(id))) -> Collider {
  // Check if empty using dict.size (O(1)) instead of converting to list
  case dict.size(bodies) {
    0 -> {
      // Default bounds for empty world
      collider.box(
        min: vec3.Vec3(-100.0, -100.0, -1.0),
        max: vec3.Vec3(100.0, 100.0, 1.0),
      )
    }
    _ -> {
      // Find min/max positions including body bounding radius
      let init_min = vec3.Vec3(1.0e10, 1.0e10, 1.0e10)
      let init_max = vec3.Vec3(-1.0e10, -1.0e10, -1.0e10)

      let #(min_3d, max_3d) =
        dict.fold(bodies, #(init_min, init_max), fn(acc, _id, body) {
          let #(current_min, current_max) = acc

          let radius = body.bounding_radius(body)
          let body_min =
            vec3.Vec3(
              body.position.x -. radius,
              body.position.y -. radius,
              body.position.z -. radius,
            )
          let body_max =
            vec3.Vec3(
              body.position.x +. radius,
              body.position.y +. radius,
              body.position.z +. radius,
            )

          #(
            vec3.Vec3(
              float.min(current_min.x, body_min.x),
              float.min(current_min.y, body_min.y),
              float.min(current_min.z, body_min.z),
            ),
            vec3.Vec3(
              float.max(current_max.x, body_max.x),
              float.max(current_max.y, body_max.y),
              float.max(current_max.z, body_max.z),
            ),
          )
        })

      // Add padding to bounds
      let padding = 10.0
      collider.box(
        min: vec3.Vec3(
          min_3d.x -. padding,
          min_3d.y -. padding,
          min_3d.z -. padding,
        ),
        max: vec3.Vec3(
          max_3d.x +. padding,
          max_3d.y +. padding,
          max_3d.z +. padding,
        ),
      )
    }
  }
}

// =============================================================================
// Raycasting
// =============================================================================

/// Cast a ray and find the first body it hits
///
/// Returns the closest hit along the ray, or None if no hit.
pub fn raycast(
  bodies: Dict(id, Body(id)),
  spatial_index: option.Option(SpatialIndex(id)),
  origin: Vec3(Float),
  direction: Vec3(Float),
  max_distance: Float,
  layer_mask: option.Option(Int),
) -> option.Option(RaycastHit(id)) {
  // Normalize direction
  let dir_length = vec3f.length(direction)
  let dir_normalized = case dir_length >. 0.0001 {
    True -> vec3f.scale(direction, 1.0 /. dir_length)
    False -> vec3.Vec3(1.0, 0.0, 0.0)
  }

  // Get candidates based on spatial index
  let candidates = case spatial_index {
    option.None -> dict.to_list(bodies)
    option.Some(BVHIndex(tree)) -> {
      // Query along ray path (use radius query at endpoint)
      let endpoint =
        vec3f.add(origin, vec3f.scale(dir_normalized, max_distance))
      let center_3d =
        vec3.Vec3(
          { origin.x +. endpoint.x } /. 2.0,
          { origin.y +. endpoint.y } /. 2.0,
          { origin.z +. endpoint.z } /. 2.0,
        )
      let radius = max_distance /. 2.0 +. 10.0
      // Extra padding
      let results = bvh.query_radius(tree, center_3d, radius)
      extract_bodies_from_results(results, bodies)
    }
    option.Some(GridIndex(grid)) -> {
      let endpoint =
        vec3f.add(origin, vec3f.scale(dir_normalized, max_distance))
      let center_3d =
        vec3.Vec3(
          { origin.x +. endpoint.x } /. 2.0,
          { origin.y +. endpoint.y } /. 2.0,
          { origin.z +. endpoint.z } /. 2.0,
        )
      let radius = max_distance /. 2.0 +. 10.0
      let results = grid.query_radius(grid, center_3d, radius)
      extract_bodies_from_results(results, bodies)
    }
  }

  // Test ray against each candidate
  let hits =
    list.filter_map(candidates, fn(pair) {
      let #(id, candidate_body) = pair

      // Check layer mask if provided
      case layer_mask {
        option.Some(mask) ->
          case int.bitwise_and(candidate_body.layer, mask) == 0 {
            True -> Error(Nil)
            False ->
              raycast_body(
                origin,
                dir_normalized,
                max_distance,
                id,
                candidate_body,
              )
          }
        option.None ->
          raycast_body(origin, dir_normalized, max_distance, id, candidate_body)
      }
    })

  // Find closest hit
  list.fold(
    hits,
    option.None,
    fn(closest: option.Option(RaycastHit(id)), hit: RaycastHit(id)) {
      case closest {
        option.None -> option.Some(hit)
        option.Some(closest_hit) ->
          case hit.distance <. closest_hit.distance {
            True -> option.Some(hit)
            False -> closest
          }
      }
    },
  )
}

/// Test ray against a single body's collider
fn raycast_body(
  origin: Vec3(Float),
  direction: Vec3(Float),
  max_distance: Float,
  body_id: id,
  body_data: Body(id),
) -> Result(RaycastHit(id), Nil) {
  // Simple ray-sphere intersection (works for all collider types using bounding radius)
  let collider_center = body_data.position
  let radius = body.bounding_radius(body_data)

  // Vector from ray origin to sphere center
  let to_center = vec3f.subtract(collider_center, origin)

  // Project to_center onto ray direction
  let projection = vec3f.dot(to_center, direction)

  // If projection is negative, sphere is behind ray
  case projection <. 0.0 {
    True -> Error(Nil)
    False -> {
      // Find closest point on ray to sphere center
      let closest_point = vec3f.add(origin, vec3f.scale(direction, projection))
      let distance_to_center = vec3f.distance(closest_point, collider_center)

      // Check if ray intersects sphere
      case distance_to_center <=. radius {
        False -> Error(Nil)
        True -> {
          // Calculate intersection distance
          let offset_sq =
            radius *. radius -. distance_to_center *. distance_to_center
          let offset = case offset_sq {
            0.0 -> 0.0
            _ -> {
              case offset_sq |> float.square_root {
                Ok(o) -> o
                Error(_) -> 0.0
              }
            }
          }
          let hit_distance = projection -. offset

          // Check if hit is within max distance
          case hit_distance >=. 0.0 && hit_distance <=. max_distance {
            False -> Error(Nil)
            True -> {
              let hit_point =
                vec3f.add(origin, vec3f.scale(direction, hit_distance))
              let normal_vec = vec3f.subtract(hit_point, collider_center)
              let normal_length = vec3f.length(normal_vec)
              let normal = case normal_length >. 0.0001 {
                True -> vec3f.scale(normal_vec, 1.0 /. normal_length)
                False -> vec3.Vec3(1.0, 0.0, 0.0)
              }

              Ok(RaycastHit(
                body_id: body_id,
                body: body_data,
                point: hit_point,
                normal: normal,
                distance: hit_distance,
              ))
            }
          }
        }
      }
    }
  }
}
