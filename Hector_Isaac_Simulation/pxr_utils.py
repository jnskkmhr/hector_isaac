from pxr import Gf, UsdPhysics, UsdGeom, UsdShade, Sdf, Usd, PhysxSchema
import numpy as np

def createFixedJoint(
    stage: Usd.Stage,
    path: str,
    body_path1: str = None,
    body_path2: str = None,
) -> UsdPhysics.FixedJoint:
    """
    Creates a fixed joint between two bodies.

    Args:
        stage (Usd.Stage): The stage to create the fixed joint.
        path (str): The path of the fixed joint.
        body_path1 (str, optional): The path of the first body.
        body_path2 (str, optional): The path of the second body.

    Returns:
        UsdPhysics.FixedJoint: The fixed joint.
    """

    # Create fixed joint
    joint = UsdPhysics.FixedJoint.Define(stage, path)
    # Set body targets
    if body_path1 is not None:
        joint.CreateBody0Rel().SetTargets([body_path1])
    if body_path2 is not None:
        joint.CreateBody1Rel().SetTargets([body_path2])

    if (body_path1 is not None) and (body_path2 is not None):
        # Get from the simulation the position/orientation of the bodies
        body_1_prim = stage.GetPrimAtPath(body_path1)
        body_2_prim = stage.GetPrimAtPath(body_path2)
        xform_body_1 = UsdGeom.Xformable(body_1_prim)
        xform_body_2 = UsdGeom.Xformable(body_2_prim)
        transform_body_1 = xform_body_1.ComputeLocalToWorldTransform(0.0)
        transform_body_2 = xform_body_2.ComputeLocalToWorldTransform(0.0)
        t12 = np.matmul(
            np.linalg.inv(transform_body_1).T, np.array(transform_body_2).T
        ).T
        translate_body_12 = Gf.Vec3f([t12[3][0], t12[3][1], t12[3][2]])
        Q_body_12 = Gf.Transform(Gf.Matrix4d(t12.tolist())).GetRotation().GetQuat()

        # Set the transform between the bodies inside the joint
        joint.CreateLocalPos0Attr().Set(translate_body_12)
        joint.CreateLocalPos1Attr().Set(Gf.Vec3d([0, 0, 0]))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(Q_body_12))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    else:
        # Set the transform between the bodies inside the joint
        joint.CreateLocalPos0Attr().Set(Gf.Vec3d([0, 0, 0]))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3d([0, 0, 0]))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

    return joint