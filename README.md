# expresso

Lightweight 3D physics engine for Gleam game development.

[![Package Version](https://img.shields.io/hexpm/v/expresso)](https://hex.pm/packages/expresso)
[![Hex Docs](https://img.shields.io/badge/hex-docs-ffaff3)](https://hexdocs.pm/expresso/)

## Overview

Expresso provides deterministic 3D rigid body physics using position-based dynamics with quaternion rotations and spatial acceleration (BVH/Grid). Designed for games, simulations, and particle systems.

**Key Features:**
- Multiple collider shapes (Sphere, Box, Capsule, Cylinder)
- Quaternion-based 3D rotations with automatic torque
- Spatial acceleration (BVH for dynamic scenes, Grid for particles)
- Collision layers and filtering
- Raycasting and spatial queries
- Trigger zones and collision events
- Continuous collision detection (CCD)
- Friction and restitution
- Deterministic and multi-target (Erlang + JavaScript)

## Installation

```sh
gleam add expresso
```

## Quick Start

```gleam
import expresso/body
import expresso/world
import vec/vec3

pub fn main() {
  // Create world
  let world = world.new(gravity: vec3.Vec3(0.0, -9.8, 0.0))
  
  // Add bodies
  let world =
    world
    |> world.add_body(body.new_sphere("ball", vec3.Vec3(0.0, 5.0, 0.0), radius: 0.5))
    |> world.add_body(body.new_box("platform", vec3.Vec3(0.0, 0.0, 0.0), half_extents: vec3.Vec3(5.0, 0.5, 5.0)))
  
  // Step physics
  let #(world, events) = world.step(world, delta_time: 1.0 /. 60.0)
  
  // Get updated positions
  let assert Ok(ball) = world.get_body(world, "ball")
}
```

## Core Concepts

### Creating Bodies

```gleam
// Sphere - best for projectiles and particles
let bullet = body.new_sphere("bullet", vec3.Vec3(0.0, 0.0, 0.0), radius: 0.25)

// Box - best for walls and platforms
let wall = body.new_box("wall", vec3.Vec3(0.0, 0.0, 0.0), 
                        half_extents: vec3.Vec3(10.0, 1.0, 5.0))

// Capsule - best for character controllers
let player = body.new_capsule("player", vec3.Vec3(0.0, 2.0, 0.0), 
                              height: 2.0, radius: 0.5)

// Cylinder - best for pillars and towers
import spatial/collider
let tower = body.new("tower", vec3.Vec3(5.0, 0.0, 0.0),
                     collider.cylinder(center: vec3.Vec3(0.0, 0.0, 0.0), 
                                      radius: 1.0, height: 3.0))
```

### Body Types

```gleam
// Dynamic - affected by physics
let dynamic = body.new_sphere("ball", vec3.Vec3(0.0, 5.0, 0.0), radius: 0.5)

// Kinematic - player-controlled, not affected by forces
let kinematic = 
  body.new_capsule("player", vec3.Vec3(0.0, 0.0, 0.0), height: 2.0, radius: 0.5)
  |> body.kinematic()

// Static - doesn't move
let static = 
  body.new_box("ground", vec3.Vec3(0.0, 0.0, 0.0), half_extents: vec3.Vec3(10.0, 0.5, 10.0))
  |> body.with_mass(0.0)
```

### Rotation

```gleam
import quaternion

// Set rotation
let rotation = quaternion.from_axis_angle(vec3.Vec3(0.0, 1.0, 0.0), 1.57)
let box = 
  body.new_box("box", vec3.Vec3(0.0, 0.0, 0.0), half_extents: vec3.Vec3(1.0, 1.0, 1.0))
  |> body.with_rotation(rotation)

// Add angular velocity
let spinner = 
  body.new_sphere("spinner", vec3.Vec3(0.0, 0.0, 0.0), radius: 1.0)
  |> body.with_angular_velocity(vec3.Vec3(0.0, 0.0, 2.0))
```

### World Management

```gleam
// Create and configure world
let world = 
  world.new(gravity: vec3.Vec3(0.0, -9.8, 0.0))
  |> world.with_iterations(10)
  |> world.with_restitution(0.5)

// Update body each frame
let world = 
  world.update_body(world, "player", fn(body) {
    body.with_velocity(player_velocity)
  })

// Step physics
let #(world, events) = world.step(world, delta_time: 0.016)
```

### Spatial Queries

```gleam
// Find bodies near a point
let nearby = world.query_radius(world, center: position, radius: 5.0)

// Find bodies in a region
let visible = world.query_region(world, 
                                 min: vec3.Vec3(-10.0, -10.0, -10.0),
                                 max: vec3.Vec3(10.0, 10.0, 10.0))

// Find nearest body
case world.query_nearest(world, point: player_pos) {
  Some(#(id, body, distance)) -> // Use nearest
  None -> // No bodies found
}
```

### Collision Layers

```gleam
// Set layer and collision mask
let player_bullet =
  body.new_sphere("bullet", vec3.Vec3(0.0, 0.0, 0.0), 0.2)
  |> body.with_layer(body.layer_projectile)
  |> body.with_collision_mask(body.combine_layers([
       body.layer_enemy,
       body.layer_environment,
     ]))
```

**Predefined layers:** `layer_default`, `layer_player`, `layer_enemy`, `layer_projectile`, `layer_environment`, `layer_trigger`, `layer_all`

### Raycasting

```gleam
case world.raycast(world, 
                   origin: player_pos,
                   direction: vec3.Vec3(1.0, 0.0, 0.0),
                   max_distance: 100.0,
                   layer_mask: option.None) {
  option.Some(hit) -> {
    // hit.body_id, hit.distance, hit.point, hit.normal
  }
  option.None -> {}
}
```

### Collision Events

```gleam
let #(world, events) = world.step(world, delta_time: 1.0 /. 60.0)

list.each(events, fn(event) {
  case event {
    world.CollisionStarted(a, b) -> // Bodies started colliding
    world.CollisionEnded(a, b) -> // Bodies separated
    world.TriggerEntered(body: b, trigger: t) -> // Body entered trigger
    world.TriggerExited(body: b, trigger: t) -> // Body left trigger
  }
})
```

### Trigger Zones

```gleam
// Non-physical sensor
let checkpoint =
  body.new_sphere("checkpoint", vec3.Vec3(10.0, 0.0, 0.0), 2.0)
  |> body.trigger()
```

### Advanced Features

```gleam
// Friction
let surface = 
  body.new_box("surface", vec3.Vec3(0.0, 0.0, 0.0), half_extents: vec3.Vec3(10.0, 0.5, 10.0))
  |> body.with_static_friction(0.6)
  |> body.with_dynamic_friction(0.4)

// Continuous collision detection (for fast objects)
let fast_bullet = 
  body.new_sphere("bullet", vec3.Vec3(0.0, 0.0, 0.0), radius: 0.1)
  |> body.with_velocity(vec3.Vec3(100.0, 0.0, 0.0))
  |> body.with_ccd(True)

// Spatial strategy selection
let world = world.with_spatial_strategy(world, world.UseBVH)  // Dynamic scenes
let world = world.with_spatial_strategy(world, world.UseGrid(cell_size: 2.0))  // Particles
```

## How It Works

Expresso uses position-based dynamics with spatial acceleration:

1. Apply forces (gravity, external)
2. Integrate velocities
3. Broad-phase collision detection (BVH/Grid)
4. Narrow-phase collision (shape intersection)
5. Resolve constraints (iterative separation)
6. Apply impulses (velocity update)

**Spatial Acceleration:**
- **BVH**: Best for dynamic scenes with moving objects
- **Grid**: Best for uniform distributions (particles, crowds)
- **Auto**: Automatically selects based on body count and distribution

## Performance

| Body Count | Strategy | Performance |
|------------|----------|-------------|
| <10 | Brute Force | Excellent (automatic) |
| 10-100 | BVH | Excellent |
| 100-1000 | BVH/Grid | Good |
| >1000 | Grid | Good |

## Limitations

Expresso does not support:
- Joints or complex constraints
- Soft bodies or cloth simulation
- Triangle mesh colliders or heightmaps
- Oriented bounding boxes (uses conservative AABBs)

For advanced features, consider full physics engines like Rapier or PhysX.

## Documentation

Full API documentation: [HexDocs](https://hexdocs.pm/expresso/)

**Key modules:**
- `expresso/body` - Body creation and management
- `expresso/world` - World simulation and queries
- `expresso/collision` - Collision detection

## Development

```sh
gleam test           # Run tests
gleam build          # Build project
gleam docs build     # Generate docs
```

## License

MIT

## Credits

Inspired by Position Based Dynamics (Müller et al.), Box2D (Erin Catto), Nature of Code (Daniel Shiffman), and the Gleam spatial library.
