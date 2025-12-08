# Mixed Vertex Morphing Tool

This small python tool showcases the *mixed Vertex Morphing parameterization* with *rigid body* parameters.\
![](/examples/3_1_rigid_body/references/figures/figx_rigid_body_obj_contour.png)

It runs the numerical *1D*-examples of the paper:\
"Mixing the explicit Vertex Morphing method with rigid body parameters for node-based shape optimization".

## Setup

Using python virtual environments or miniconda as an environment manager is recommended.

A miniconda environment can be created with the `.yml` file which stores all required python libraries:
```bash
conda env create -y -f environment.yml
```

Add the repository folder to your PYTHONPATH, e.g. by editing your bashrc:\\`export PYTHONPATH=$PYTHONPATH:$HOME/.../MixedVertexMorphingTool`

## Numerical Examples
The following numerical 1D geometry fitting optimizations of the paper are replicated:
- Section 3.1: [rigid body parameterization](/examples/3_1_rigid_body/)
- Section 4.3: [study on computational costs](/examples/4_3_mixed_costs/)
- Section 4.4: [mixed parameterization side-by-side](/examples/4_4_1_mixed_side_by_side/)
- Section 4.4: [mixed parameterization overlapping](/examples/4_4_2_mixed_overlap/)
