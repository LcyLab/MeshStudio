"""Reference-only corner UV candidates; the own generator never imports this module.

The SDK's stored face parameters and direct inverse can disagree at CAD vertices
or inconsistent pcurves. Choose by forward XYZ residual, then let the independent
CAD audit validate the result. No coordinates, mesh decisions or gates are changed.
"""
import math


def collect_reference_face_parameters(gmsh, entity, diagnostics=None, required_coordinates=None):
    tags, coordinates, native = gmsh.model.mesh.getNodes(2, entity, includeBoundary=True, returnParametricCoord=True)
    tags = list(map(int, tags)); coordinates = list(map(float, coordinates)); native = list(map(float, native))
    if len(coordinates) != 3*len(tags) or len(native) != 2*len(tags):
        raise ValueError(f"reference face {entity}: native parameter coverage incomplete")
    if not all(math.isfinite(x) for x in coordinates+native):
        raise ValueError(f"reference face {entity}: nonfinite native parameter or coordinate")
    required_coordinates = {int(tag): list(map(float, xyz))
                            for tag, xyz in (required_coordinates or {}).items()}
    if any(len(xyz) != 3 or not all(math.isfinite(x) for x in xyz)
           for xyz in required_coordinates.values()):
        raise ValueError(f"reference face {entity}: invalid required coordinate")
    for i, tag in enumerate(tags):
        if tag in required_coordinates and coordinates[3*i:3*i+3] != required_coordinates[tag]:
            raise ValueError(f"reference face {entity}: coordinate identity mismatch for node {tag}")
    inverse = list(map(float, gmsh.model.getParametrization(2, entity, coordinates)))
    if len(inverse) != 2*len(tags):
        raise ValueError(f"reference face {entity}: inverse parameter coverage incomplete")
    if not all(math.isfinite(x) for x in inverse):
        raise ValueError(f"reference face {entity}: nonfinite inverse parameter")
    native_xyz = list(map(float, gmsh.model.getValue(2, entity, native)))
    inverse_xyz = list(map(float, gmsh.model.getValue(2, entity, inverse)))
    if len(native_xyz) != len(coordinates) or len(inverse_xyz) != len(coordinates):
        raise ValueError(f"reference face {entity}: forward value coverage incomplete")
    if not all(math.isfinite(x) for x in native_xyz+inverse_xyz):
        raise ValueError(f"reference face {entity}: nonfinite forward value")
    result, residuals = {}, {}
    summary = dict(native_selected=0, inverse_selected=0, direct_only_nodes=0, maximum_native_residual=0.,
                   maximum_inverse_residual=0., maximum_selected_residual=0.)
    for i, tag in enumerate(tags):
        xyz = coordinates[3*i:3*i+3]
        native_residual = math.dist(xyz, native_xyz[3*i:3*i+3])
        inverse_residual = math.dist(xyz, inverse_xyz[3*i:3*i+3])
        use_native = native_residual <= inverse_residual
        selected = native if use_native else inverse
        residual = native_residual if use_native else inverse_residual
        if tag not in residuals or residual < residuals[tag]:
            result[tag] = selected[2*i:2*i+2]; residuals[tag] = residual
        summary['native_selected' if use_native else 'inverse_selected'] += 1
        summary['maximum_native_residual'] = max(summary['maximum_native_residual'], native_residual)
        summary['maximum_inverse_residual'] = max(summary['maximum_inverse_residual'], inverse_residual)
    # getNodes(includeBoundary=True) omits entities embedded in the face interior.
    # Their element node identities still require a checked parameter for export.
    missing = [tag for tag in required_coordinates if tag not in result]
    if missing:
        missing_xyz = [x for tag in missing for x in required_coordinates[tag]]
        direct = list(map(float, gmsh.model.getParametrization(2, entity, missing_xyz)))
        if len(direct) != 2*len(missing) or not all(math.isfinite(x) for x in direct):
            raise ValueError(f"reference face {entity}: incomplete or nonfinite embedded parameter")
        direct_xyz = list(map(float, gmsh.model.getValue(2, entity, direct)))
        if len(direct_xyz) != len(missing_xyz) or not all(math.isfinite(x) for x in direct_xyz):
            raise ValueError(f"reference face {entity}: incomplete or nonfinite embedded forward value")
        for i, tag in enumerate(missing):
            result[tag] = direct[2*i:2*i+2]
            residuals[tag] = math.dist(required_coordinates[tag], direct_xyz[3*i:3*i+3])
            summary['maximum_inverse_residual'] = max(summary['maximum_inverse_residual'], residuals[tag])
        summary['direct_only_nodes'] = len(missing)
    summary['maximum_selected_residual'] = max(residuals.values(), default=0.)
    if diagnostics is not None:
        diagnostics.update(summary)
    return result
