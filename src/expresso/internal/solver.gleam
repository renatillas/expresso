//// Position-based constraint solver for 2D physics.
////
//// This module implements a position-based dynamics (PBD) solver that resolves
//// collisions by iteratively separating overlapping bodies and applying impulses
//// for realistic collision response.
////
//// ## Algorithm
////
//// The solver works in two phases:
////
//// 1. **Position Resolution** - Iteratively push apart overlapping bodies
//// 2. **Velocity Resolution** - Apply impulses for bounce and realism
////
//// ## Position-Based Dynamics
////
//// Position-based dynamics is a simulation method that:
//// - Directly modifies positions (not forces)
//// - Iteratively converges to a solution
//// - Is unconditionally stable (no explosions!)
//// - Produces deterministic results
////
//// ### How It Works
////
//// ```
//// For each iteration:
////   For each collision contact:
////     1. Calculate penetration depth
////     2. Compute correction based on inverse mass ratio
////     3. Push bodies apart along contact normal
//// ```
////
//// More iterations = more accurate, but slower.
//// Typical values: 3-5 iterations for games, 10+ for stable stacks.
////
//// ## Restitution (Bounciness)
////
//// The `restitution` parameter controls how bouncy collisions are:
////
//// - **0.0** - Perfectly inelastic (no bounce)
//// - **0.5** - Moderate bounce
//// - **1.0** - Perfectly elastic (full energy conservation)
////
//// ## Usage
////
//// Most users won't call this module directly - the `world` module handles
//// constraint solving automatically. However, you can configure the solver:
////
//// ```gleam
//// import expresso/world
////
//// let world = world.new(gravity: vec2.Vec2(0.0, -9.8))
////   |> world.with_iterations(10)       // More stable
////   |> world.with_restitution(0.5)     // Moderate bounce
//// ```

import expresso/body.{type Body}
import expresso/collision.{type Contact}
import gleam/dict.{type Dict}
import gleam/list
import vec/vec3
import vec/vec3f

/// Iteratively resolve position constraints
///
/// Separates overlapping bodies by applying correction impulses.
/// Uses 3-5 iterations for stable results.
///
/// Relaxation factor controls how much of the penetration to correct per iteration:
/// - 1.0 = correct 100% (fast but can be unstable with many overlaps)
/// - 0.8 = correct 80% (more stable, recommended for crowds)
/// - 0.5 = correct 50% (very stable but slower convergence)
pub fn solve_position_constraints(
  bodies: Dict(id, Body(id)),
  contacts: List(Contact(id)),
  iterations: Int,
) -> Dict(id, Body(id)) {
  // Use relaxation factor for stability with dense crowds
  // 0.6 = correct 60% per iteration, preventing overcorrection
  let relaxation = 0.6

  // Iteratively apply corrections
  list.range(0, iterations - 1)
  |> list.fold(bodies, fn(current_bodies, _iteration) {
    // Apply all contacts in sequence
    list.fold(contacts, current_bodies, fn(bodies_acc, contact) {
      apply_contact_correction(bodies_acc, contact, relaxation)
    })
  })
}

/// Apply position correction for a single contact with relaxation
fn apply_contact_correction(
  bodies: Dict(id, Body(id)),
  contact: Contact(id),
  relaxation: Float,
) -> Dict(id, Body(id)) {
  // Get both bodies - return early if either is missing
  case dict.get(bodies, contact.body_a), dict.get(bodies, contact.body_b) {
    Ok(body_a), Ok(body_b) -> {
      // Skip if either body is a trigger (sensors don't participate in collision resolution)
      case body_a.is_trigger || body_b.is_trigger {
        True -> bodies
        False -> {
          // Calculate inverse masses
          let inv_mass_a = body.inverse_mass(body_a)
          let inv_mass_b = body.inverse_mass(body_b)
          let total_inv_mass = inv_mass_a +. inv_mass_b

          // Skip if both bodies are kinematic (infinite mass)
          case total_inv_mass <. 0.0001 {
            True -> bodies
            False -> {
              // Calculate correction magnitude with relaxation factor
              // Relaxation < 1.0 makes solver more stable with many overlapping bodies
              let correction_magnitude =
                contact.penetration *. relaxation /. total_inv_mass

              // Calculate correction vectors
              let correction_a =
                vec3f.scale(
                  contact.normal,
                  -1.0 *. correction_magnitude *. inv_mass_a,
                )
              let correction_b =
                vec3f.scale(contact.normal, correction_magnitude *. inv_mass_b)

              // Apply corrections
              let new_body_a =
                body.Body(
                  ..body_a,
                  position: vec3f.add(body_a.position, correction_a),
                )
              let new_body_b =
                body.Body(
                  ..body_b,
                  position: vec3f.add(body_b.position, correction_b),
                )

              // Update bodies dict
              bodies
              |> dict.insert(contact.body_a, new_body_a)
              |> dict.insert(contact.body_b, new_body_b)
            }
          }
        }
      }
    }
    _, _ -> bodies
  }
}

/// Apply velocity-level impulses for collision response
///
/// This gives bodies a "bounce" when they collide.
/// Restitution controls how bouncy collisions are (0.0 = no bounce, 1.0 = perfectly elastic).
pub fn solve_velocity_constraints(
  bodies: Dict(id, Body(id)),
  contacts: List(Contact(id)),
  restitution: Float,
) -> Dict(id, Body(id)) {
  list.fold(contacts, bodies, fn(bodies_acc, contact) {
    apply_velocity_impulse(bodies_acc, contact, restitution)
  })
}

/// Apply velocity impulse for a single contact (with torque)
fn apply_velocity_impulse(
  bodies: Dict(id, Body(id)),
  contact: Contact(id),
  restitution: Float,
) -> Dict(id, Body(id)) {
  // Get both bodies - return early if either is missing
  case dict.get(bodies, contact.body_a), dict.get(bodies, contact.body_b) {
    Ok(body_a), Ok(body_b) -> {
      // Skip if either body is a trigger (sensors don't participate in collision resolution)
      case body_a.is_trigger || body_b.is_trigger {
        True -> bodies
        False -> {
          // Calculate relative velocity at contact point
          let relative_velocity =
            vec3f.subtract(body_b.velocity, body_a.velocity)
          let velocity_along_normal =
            vec3f.dot(relative_velocity, contact.normal)

          // Bodies are separating, no impulse needed
          case velocity_along_normal >=. 0.0 {
            True -> bodies
            False -> {
              // Calculate impulse magnitude
              let inv_mass_a = body.inverse_mass(body_a)
              let inv_mass_b = body.inverse_mass(body_b)
              let total_inv_mass = inv_mass_a +. inv_mass_b

              case total_inv_mass <. 0.0001 {
                True -> bodies
                False -> {
                  // J = -(1 + e) * vn / (1/ma + 1/mb)
                  let impulse_magnitude =
                    -1.0
                    *. { 1.0 +. restitution }
                    *. velocity_along_normal
                    /. total_inv_mass

                  let impulse = vec3f.scale(contact.normal, impulse_magnitude)

                  // Calculate torque for body A: torque = r × F
                  // where r is the vector from center of mass to contact point
                  let r_a = vec3f.subtract(contact.point, body_a.position)
                  let torque_a = vec3f.cross(r_a, vec3f.scale(impulse, -1.0))
                  let angular_impulse_a =
                    apply_torque(body_a, torque_a, inv_mass_a)

                  // Calculate torque for body B
                  let r_b = vec3f.subtract(contact.point, body_b.position)
                  let torque_b = vec3f.cross(r_b, impulse)
                  let angular_impulse_b =
                    apply_torque(body_b, torque_b, inv_mass_b)

                  // Apply normal impulses
                  let body_a_after_normal =
                    body.Body(
                      ..body_a,
                      velocity: vec3f.add(
                        body_a.velocity,
                        vec3f.scale(impulse, -1.0 *. inv_mass_a),
                      ),
                      angular_velocity: vec3f.add(
                        body_a.angular_velocity,
                        angular_impulse_a,
                      ),
                    )
                  let body_b_after_normal =
                    body.Body(
                      ..body_b,
                      velocity: vec3f.add(
                        body_b.velocity,
                        vec3f.scale(impulse, inv_mass_b),
                      ),
                      angular_velocity: vec3f.add(
                        body_b.angular_velocity,
                        angular_impulse_b,
                      ),
                    )

                  // Apply friction impulses
                  let #(new_body_a, new_body_b) =
                    apply_friction_impulse(
                      body_a_after_normal,
                      body_b_after_normal,
                      contact,
                      impulse_magnitude,
                    )

                  // Update bodies
                  bodies
                  |> dict.insert(contact.body_a, new_body_a)
                  |> dict.insert(contact.body_b, new_body_b)
                }
              }
            }
          }
        }
      }
    }
    _, _ -> bodies
  }
}

/// Apply torque to get angular impulse
/// Uses simplified moment of inertia based on bounding radius
fn apply_torque(
  body_val: Body(id),
  torque: vec3.Vec3(Float),
  inverse_mass: Float,
) -> vec3.Vec3(Float) {
  // For kinematic bodies (infinite mass), no angular impulse
  case inverse_mass <. 0.0001 {
    True -> vec3.Vec3(0.0, 0.0, 0.0)
    False -> {
      // Calculate simplified moment of inertia
      // For a sphere: I = (2/5) * m * r^2
      // Using bounding radius as approximation
      let mass = 1.0 /. inverse_mass
      let radius = body.bounding_radius(body_val)
      let moment_of_inertia = 0.4 *. mass *. radius *. radius

      // Prevent division by zero
      case moment_of_inertia <. 0.0001 {
        True -> vec3.Vec3(0.0, 0.0, 0.0)
        False -> {
          // Angular impulse = torque / moment_of_inertia
          vec3f.scale(torque, 1.0 /. moment_of_inertia)
        }
      }
    }
  }
}

/// Apply friction impulses using Coulomb friction model
fn apply_friction_impulse(
  body_a: Body(id),
  body_b: Body(id),
  contact: Contact(id),
  normal_impulse_magnitude: Float,
) -> #(Body(id), Body(id)) {
  // Calculate relative velocity at contact point
  let relative_velocity = vec3f.subtract(body_b.velocity, body_a.velocity)

  // Calculate tangential velocity (perpendicular to normal)
  let velocity_along_normal = vec3f.dot(relative_velocity, contact.normal)
  let normal_velocity = vec3f.scale(contact.normal, velocity_along_normal)
  let tangential_velocity = vec3f.subtract(relative_velocity, normal_velocity)

  // Get tangential speed
  let tangential_speed = vec3f.length(tangential_velocity)

  // If tangential velocity is negligible, no friction needed
  case tangential_speed <. 0.001 {
    True -> #(body_a, body_b)
    False -> {
      // Tangent direction (normalized)
      let tangent = vec3f.normalize(tangential_velocity)

      // Combine friction coefficients (use average for now, could use other combinations)
      let static_friction =
        { body_a.static_friction +. body_b.static_friction } /. 2.0
      let dynamic_friction =
        { body_a.dynamic_friction +. body_b.dynamic_friction } /. 2.0

      // Calculate masses
      let inv_mass_a = body.inverse_mass(body_a)
      let inv_mass_b = body.inverse_mass(body_b)
      let total_inv_mass = inv_mass_a +. inv_mass_b

      case total_inv_mass <. 0.0001 {
        True -> #(body_a, body_b)
        False -> {
          // Static friction: try to completely stop tangential motion
          let static_impulse_magnitude = tangential_speed /. total_inv_mass

          // Dynamic friction: proportional to normal force
          let dynamic_impulse_magnitude =
            dynamic_friction *. normal_impulse_magnitude

          // Use static friction if it's sufficient, otherwise use dynamic
          let friction_impulse_magnitude = case
            static_impulse_magnitude
            <. static_friction *. normal_impulse_magnitude
          {
            True -> static_impulse_magnitude
            False -> dynamic_impulse_magnitude
          }

          // Friction opposes motion (negative tangent direction)
          let friction_impulse =
            vec3f.scale(tangent, -1.0 *. friction_impulse_magnitude)

          // Apply friction impulses
          let new_body_a =
            body.Body(
              ..body_a,
              velocity: vec3f.add(
                body_a.velocity,
                vec3f.scale(friction_impulse, -1.0 *. inv_mass_a),
              ),
            )
          let new_body_b =
            body.Body(
              ..body_b,
              velocity: vec3f.add(
                body_b.velocity,
                vec3f.scale(friction_impulse, inv_mass_b),
              ),
            )

          #(new_body_a, new_body_b)
        }
      }
    }
  }
}
