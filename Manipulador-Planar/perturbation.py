import pybullet as p
import math


class PerturbationTester:
    """Test robot response to disturbances (payload changes)."""

    def __init__(self):
        self.payload_bodies = []
        self.payload_mass = 0.0
        self.constraint_id = None

    def attach_payload_to_ee(self, robot_arm, mass=0.5, radius=0.03):
        """Attach a spherical payload to the end-effector."""
        shape = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=radius,
            rgbaColor=[1, 0.5, 0, 0.7]
        )
        collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)

        # Create the payload body
        payload_id = p.createMultiBody(
            baseMass=mass,
            baseVisualShapeIndex=shape,
            baseCollisionShapeIndex=collision_shape,
            basePosition=[0, 0, 0.5]
        )

        # Attach to end-effector (link index 1 is link2, the last link)
        try:
            self.constraint_id = p.createConstraint(
                robot_arm.body, 1,
                payload_id, -1,
                p.JOINT_FIXED,
                [0, 0, 0], [0, 0, 0], [0, 0, 0]
            )
            self.payload_mass = mass
            self.payload_bodies.append(payload_id)
            return payload_id
        except Exception as e:
            print(f"Failed to attach payload: {e}")
            p.removeBody(payload_id)
            return None

    def detach_payload(self):
        """Detach the payload from the end-effector."""
        if self.constraint_id is not None:
            try:
                p.removeConstraint(self.constraint_id)
            except Exception:
                pass
        self.constraint_id = None

        for body_id in self.payload_bodies:
            try:
                p.removeBody(body_id)
            except Exception:
                pass
        self.payload_bodies = []
        self.payload_mass = 0.0

    def change_payload_mass(self, new_mass, robot_arm):
        """Change the mass of attached payload (simulate different weights)."""
        if self.payload_bodies:
            for body_id in self.payload_bodies:
                try:
                    # Remove and recreate the body with new mass
                    p.removeBody(body_id)
                except Exception:
                    pass

            self.payload_bodies = []
            if new_mass > 0 and robot_arm is not None:
                # Create new payload with updated mass
                self.attach_payload_to_ee(robot_arm, mass=new_mass)
