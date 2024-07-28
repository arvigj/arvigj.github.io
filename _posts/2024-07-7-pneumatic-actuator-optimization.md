---
layout: post
title: Cascaded Shape Optimization using PolyFEM
date: 2024-07-07 11:12:00-0400
description: A tutorial for setting up shape optimization of pneumatic robots
categories: optimization
related_posts: false
---

Cascaded Shape Optimization using PolyFEM (Draft In Progress)
==================
***A tutorial for setting up shape optimization of pneumatic robots, with [full examples](https://github.com/arvigj/pneumatic-actuator-design)***

In this tutorial, I'll be outlining how to setup the shape optimization from the paper ['Soft Pneumatic Actuator Design using Differentiable Simulation'](https://cims.nyu.edu/gcl/papers/2024-pneumatic.pdf). I'll be assuming familiarity with the paper, including the finite element method and optimization in general. For a background into these topics, there will be a blog post in the future. We will be using the differentiable finite element simulator [PolyFEM](https://github.com/polyfem/polyfem), which has support for high order basis, a multitude of material models, physical energies and state-of-the-art contact/friction ([IPC](https://github.com/ipc-sim/ipc-toolkit)). The cascading algorithm is implemented in Python and can be found [with the configuration files](https://github.com/arvigj/pneumatic-actuator-design). [MMG](https://www.mmgtools.org) is also required for interior remeshing.

## Simulation Setup

Readers should take a look at the [PolyFEM docs](https://polyfem.github.io/tutorials/getting_started/) for complete tutorials on setting up a simulation, with a full breakdown of features, options for contact, solvers, output, etc. For this section, we will focus on some requirements for the simulation: the mesh, surface selections and boundary conditions.

### Mesh

A finite element simulation requires a volumetric mesh. This can be created from a surface mesh using [fTetWild](https://github.com/wildmeshing/fTetWild), which exposes a lot of control over mesh quality and is very robust. One drawback is that the volumetric mesh surface will be within an epsilon tolerance of the input surface mesh (epsilon can be configured, but the algorithm will take longer to run if epsilon is small). This is only really a problem when designing object with flat features, because after meshing they will no longer be truly flat. In these cases we recommend using [TetGen]() if the object is simple.

Alternatively, if designed in CAD, volumetric meshes are directly available.

### Surface selections

The easiest way to select surfaces is by using the built in selection tools of PolyFEM, whereby you can specify surfaces bounded by geometric primitives (box, sphere). When dealing with more complicated geometries, one might want to select two sides of a thin wall, in which case these don't provide the necessary granularity. For this, we designed a utility which takes in the volumetric mesh and (watertight) surface meshes enclosing surfaces of interest and creates a single file of relevant surface selections. This can be found in `pneumatic-actuator-design/setup_scene_3d` and compiled as you would any cmake project. The benefit of this system is that when the objects are designed originally, whether in CAD or a modelling software like Blender, one can take the surface parts, scale them up to enclose the surfaces of interest and export these. The selections will then closely follow the surface and can be very precise. The output of this utility is a `.txt` file mapping surface triangles to boundary ids, which can be directly loaded into PolyFEM. Please see the README in the repository for how to use it.

### Boundary conditions

Once you've marked the surface selections and specified them in the json, you can use them as boundary conditions.

```
"boundary_conditions": {
    "dirichlet_boundary": [{
        "id": 0,
        "value": [0, 0, 0]
    }],
    "pressure_boundary": [{
        "id": 1,
        "value": "min(-4000 * t, -4000)"
    }]
}
```

Dirichlet boundary conditions require a specification of a displacement vector $$[x, y, z]$$ whereas pressure boundary conditions only require a pressure scalar that is applied normal to the surface $$p \, \hat{n}$$ (the convention used is that negative values represent incedent pressure). PolyFEM also accepts functions of $$x, y, z, t$$ as boundary condition values, allowing pressure to be varied over time as the linear ramp `min(-4000 * t, -4000)`. There are other functions available and boundary conditions can also be specified per timestep (see worm example), but PolyFEM lacks more robust specification for control.

**Important Note About Simulation:** For mathematical reasons, the boundary loop of a surface with applied pressure must be either held constant with Dirichlet b.c. or the whole surface must be closed.

## Optimization Setup

Once the simulation is setup, setting up the optimization requires specifying:
1. The parameters being optimized and their starting values. 
2. The parametrizations mapping the parameters to simulation variables, i.e. quantities that the simulation directly uses. In the case of simply optimizing vertex positions, the parametrization is the identity map.
3. The optimization objectives, which can be a list of summed functionals. While there are many of them and new ones are constantly added in PolyFEM, the ones we will detail are:
    * `mesh-target` -- 
    ```
    {
        "type": "mesh-target",
        "state": 0,
        "surface_selection": [
            4
        ],
        "mesh_path": "cylinder_smaller_target_v2.obj",
        "delta": 0.001
    }
    ```
    * `log_contact_force_norm`
    * `layer_thickness`
    * `transient_integral` -- This objective is needed to wrap all of our other objectives, which are effectively static objectives (for a given displacement at time t). Through this objective, one can specify a timestep to evaluate the static objectives or evaluate them over the course of the simulation. The syntax looks as follows, with the static objective being surface matching to a mesh evaluated at time steps 53 and 54:
    ```
    {
        "type": "transient_integral",
        "weight": -1e4,
        "integral_type": "steps",
        "steps": [
            53,
            54
        ],
        "state": 0,
        "static_objective": {
            "type": "mesh-target",
            ...
        }
    }
    ```
    * `collision_barrier` -- Since we're using a quadratic barrier in our `layer_thickness` objective, we might want to add a "hard" constraint to prevent surfaces going through one another. If we keep the support of this objective small, we can ensure that the optimization does not reach an invalid state while preventing this objective from really contributing to the energy. This objective is not crucial to the optimization and can be left out.

Setting weights for each objective is nontrivial and especially with multiple competing objectives, may require a few rounds of weight tuning.

## Cascaded Optimization

At this point, you can add the new optimization info to the `OPTIMIZATIONS` dict in `cascaded_optimization.py`. Using a key `<example_key>`, the necessary fields are:

`base_path`: The base directory for the optimization files, where the results will be stored.

`state_path`: The filename of the simulation json.

`run_path`: The filename of the optimization json.

`num_control_points`: For each of the surfaces being optimized, this specified the degrees of freedom for each optimization step. It is zero indexed starting with the first parameter in the optimization json. Typically, you can start this with a low number that translates the shape first, then double the degrees of freedom iteratively.

`num_iters`: Specifies the number of iterations for each optimization step.

`opt_mesh_idx`: Specifies the index of the mesh being indexed in the simulation json, in the `geometry` section.

`aux_files`: Specifies the auxiliary files that are copied to the run directory, such as mesh targets, etc.

`threads`: Number of threads that the forward simulation is allowed.

The optimization can then be executed by the following command:

`python cascaded_optimization.py <example_key> <polyfem_build_dir> <mmg_build_dir> `

This will run the optimization and save the results in the current working directory.

