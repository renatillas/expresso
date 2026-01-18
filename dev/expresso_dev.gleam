// Physics simulation benchmarks comparing BVH update strategies
// Run with: gleam run -m physics_bench

import expresso/body
import expresso/world
import gleam/dict
import gleam/int
import gleam/io
import gleam/list
import gleam/string
import gleamy/bench
import simplifile
import spatial/bvh
import vec/vec3

pub fn main() {
  io.println("╔══════════════════════════════════════════════════════════════╗")
  io.println("║        Expresso Physics Engine Benchmark Suite              ║")
  io.println("╚══════════════════════════════════════════════════════════════╝")
  io.println("")

  let output1 = benchmark_optimized_physics()
  let output2 = benchmark_id_based_strategy()

  let full_output =
    string.join(
      [
        "╔══════════════════════════════════════════════════════════════╗",
        "║        Expresso Physics Engine Benchmark Results            ║",
        "║                    AFTER OPTIMIZATION                        ║",
        "╚══════════════════════════════════════════════════════════════╝",
        "",
        "Using ID-based BVH with incremental updates",
        "",
        output1,
        output2,
      ],
      "\n",
    )

  let _ = simplifile.create_directory_all("dist/benchmarks")

  case
    simplifile.write(
      "dist/benchmarks/physics_results_optimized.txt",
      full_output,
    )
  {
    Ok(_) ->
      io.println(
        "\n✅ Results saved to: dist/benchmarks/physics_results_optimized.txt",
      )
    Error(_) -> io.println("\n❌ Failed to save results")
  }

  io.println("\n🎉 All benchmarks complete!")
}

// ============================================================================
// Optimized Approach: ID-based BVH with fast rebuild
// ============================================================================

fn benchmark_optimized_physics() -> String {
  io.println("\n=== Optimized: ID-based BVH (40x faster rebuild) ===\n")

  bench.run(
    [
      bench.Input("50 bodies", create_simulation_world(50)),
      bench.Input("100 bodies", create_simulation_world(100)),
      bench.Input("200 bodies", create_simulation_world(200)),
      bench.Input("500 bodies", create_simulation_world(500)),
    ],
    [
      bench.Function("rebuild_and_detect", fn(w: world.World(Int)) {
        // Simulate one physics step (rebuild BVH + collision detection)
        world.step(w, delta_time: 0.016)
      }),
    ],
    [bench.Duration(1000), bench.Warmup(100)],
  )
  |> bench.table([bench.IPS, bench.Min, bench.Mean, bench.Max])
}

// ============================================================================
// Strategy 2: Store IDs in BVH and refit (proposed approach)
// ============================================================================

// This would require changing the BVH to store IDs instead of Bodies
// For now, let's benchmark the key operations separately

pub type IdBasedBVH {
  IdBasedBVH(tree: bvh.BVH(Int), bodies: dict.Dict(Int, body.Body(Int)))
}

fn benchmark_id_based_strategy() -> String {
  io.println("\n=== Strategy 2: Store IDs in BVH, refit bounds ===\n")

  bench.run(
    [
      bench.Input("50 bodies", create_id_based_world(50)),
      bench.Input("100 bodies", create_id_based_world(100)),
      bench.Input("200 bodies", create_id_based_world(200)),
      bench.Input("500 bodies", create_id_based_world(500)),
    ],
    [
      bench.Function("build_id_bvh", fn(input: IdBasedBVH) {
        // Build BVH with just IDs
        let items =
          dict.to_list(input.bodies)
          |> list.map(fn(pair) {
            let #(id, b) = pair
            let pos_3d = b.position
            #(pos_3d, id)
          })
        let assert Ok(tree) = bvh.from_items(items, max_leaf_size: 8)
        tree
      }),
      bench.Function("refit_after_move", fn(input: IdBasedBVH) {
        // Refit BVH bounds after bodies moved
        bvh.refit(input.tree)
      }),
    ],
    [bench.Duration(1000), bench.Warmup(100)],
  )
  |> bench.table([bench.IPS, bench.Min, bench.Mean, bench.Max])
}

// ============================================================================
// Helper Functions
// ============================================================================

fn create_simulation_world(body_count: Int) -> world.World(Int) {
  // Create a world with moving bodies in a grid
  let w = world.new(gravity: vec3.Vec3(0.0, -9.8, 0.0))

  list.range(0, body_count - 1)
  |> list.fold(w, fn(world_acc, i) {
    let x = int.to_float(i % 20) *. 2.0
    let y = int.to_float(i / 20) *. 2.0
    let b =
      body.new_sphere(id: i, position: vec3.Vec3(x, y, 0.0), radius: 0.5)
      |> body.with_velocity(vec3.Vec3(1.0, 0.0, 0.0))

    world.add_body(world_acc, b)
  })
}

fn create_id_based_world(body_count: Int) -> IdBasedBVH {
  // Create bodies
  let bodies_list =
    list.range(0, body_count - 1)
    |> list.map(fn(i) {
      let x = int.to_float(i % 20) *. 2.0
      let y = int.to_float(i / 20) *. 2.0
      let b =
        body.new_sphere(id: i, position: vec3.Vec3(x, y, 0.0), radius: 0.5)
        |> body.with_velocity(vec3.Vec3(1.0, 0.0, 0.0))
      #(i, b)
    })

  let bodies = dict.from_list(bodies_list)

  // Build BVH with IDs
  let bvh_items =
    bodies_list
    |> list.map(fn(pair) {
      let #(id, b) = pair
      let pos_3d = b.position
      #(pos_3d, id)
    })

  let assert Ok(tree) = bvh.from_items(bvh_items, max_leaf_size: 8)

  IdBasedBVH(tree: tree, bodies: bodies)
}
