#include "runtime/function/controller/character_controller.h"

#include "runtime/core/base/macro.h"

#include "runtime/function/framework/component/motor/motor_component.h"
#include "runtime/function/framework/world/world_manager.h"
#include "runtime/function/global/global_context.h"
#include "runtime/function/physics/physics_scene.h"

namespace Pilot
{
    CharacterController::CharacterController(const Capsule& capsule) : m_capsule(capsule)
    {
        m_rigidbody_shape                                    = RigidBodyShape();
        m_rigidbody_shape.m_geometry                         = PILOT_REFLECTION_NEW(Capsule);
        *static_cast<Capsule*>(m_rigidbody_shape.m_geometry) = m_capsule;

        m_rigidbody_shape.m_type = RigidBodyShapeType::capsule;

        Quaternion orientation;
        orientation.fromAngleAxis(Radian(Degree(90.f)), Vector3::UNIT_X);

        m_rigidbody_shape.m_local_transform =
            Transform(
                Vector3(0, 0, capsule.m_half_height + capsule.m_radius),
                orientation,
                Vector3::UNIT_SCALE);
    }

    Vector3 CharacterController::move(const Vector3& current_position, const Vector3& displacement)
    {
        std::shared_ptr<PhysicsScene> physics_scene =
            g_runtime_global_context.m_world_manager->getCurrentActivePhysicsScene().lock();
        ASSERT(physics_scene);

        std::vector<PhysicsHitInfo> hits;

        Transform world_transform = Transform(
            current_position + 0.1f * Vector3::UNIT_Z,
            Quaternion::IDENTITY,
            Vector3::UNIT_SCALE);

        Vector3 vertical_displacement   = displacement.z * Vector3::UNIT_Z;
        Vector3 horizontal_displacement = Vector3(displacement.x, displacement.y, 0.f);

        Vector3 vertical_direction   = vertical_displacement.normalisedCopy();
        Vector3 horizontal_direction = horizontal_displacement.normalisedCopy();

        Vector3 final_position = current_position;

        m_is_touch_ground = physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix(),
            Vector3::NEGATIVE_UNIT_Z,
            0.105f,
            hits);

        hits.clear();
        
        world_transform.m_position -= 0.1f * Vector3::UNIT_Z;

        // vertical pass
        if (physics_scene->sweep(
            m_rigidbody_shape,
            world_transform.getMatrix(),
            vertical_direction,
            vertical_displacement.length(),
            hits))
        {
            final_position += hits[0].hit_distance * vertical_direction;
        }
        else
        {
            final_position += vertical_displacement;
        }

        hits.clear();

        // side pass
        Vector3 side_pass_displacement = Vector3::ZERO;

        const bool clear_path = !physics_scene->sweep(m_rigidbody_shape,
                                               world_transform.getMatrix(),
                                               horizontal_direction,
                                               horizontal_displacement.length(),
                                               hits);

        const bool touch_ground = isTouchGround();

        if (clear_path)
        {
	        side_pass_displacement += horizontal_displacement;
	        world_transform.m_position += horizontal_displacement;
	        // pull obj a little bit towards Z-, if still on the ground means character going down stairs
            if (touch_ground)
	        {
		        const Vector3 forward_step_max = Vector3::NEGATIVE_UNIT_Z * m_capsule.m_half_height / 2;
		        std::vector<PhysicsHitInfo> sub_hits;
		        if (physics_scene->sweep(
			        m_rigidbody_shape, 
			        world_transform.getMatrix(), 
			        Vector3::NEGATIVE_UNIT_Z, 
			        forward_step_max.length(),
			        sub_hits))
		        {
			        side_pass_displacement += sub_hits[0].hit_distance * Vector3::NEGATIVE_UNIT_Z;
			        final_position += side_pass_displacement;
			        return final_position;
		        }
	        }
            final_position += side_pass_displacement;
            return final_position;
        }

        // pull obj a little bit towards Z+, if doesn't get blocked anymore means going upstairs
        if (touch_ground)
        {
	        const Vector3 forward_step_max = Vector3::UNIT_Z * m_capsule.m_half_height / 2;
	        world_transform.m_position += forward_step_max;

	        std::vector<PhysicsHitInfo> sub_hits;
	        if (!physics_scene->sweep(m_rigidbody_shape,
	                                  world_transform.getMatrix(),
	                                  horizontal_direction,
	                                  horizontal_displacement.length(),
	                                  sub_hits))
	        {

		        side_pass_displacement += forward_step_max + horizontal_displacement;
		        world_transform.m_position += horizontal_displacement;
		        std::vector<PhysicsHitInfo> sub_sub_hits;
		        // sweep body to the ground
		        if (physics_scene->sweep(
			        m_rigidbody_shape, 
			        world_transform.getMatrix(), 
			        Vector3::NEGATIVE_UNIT_Z,
			        forward_step_max.length(), 
			        sub_sub_hits))
		        {
			        side_pass_displacement += sub_sub_hits[0].hit_distance * Vector3::NEGATIVE_UNIT_Z;
			        final_position += side_pass_displacement;
			        return final_position;
		        }
	        }

	        world_transform.m_position -= forward_step_max;
        }

        Vector3 free_displacement = (hits[0].hit_distance - 0.001f) * horizontal_direction;

        side_pass_displacement += free_displacement;

        Vector3 surface_normal     = hits[0].hit_normal.normalisedCopy();
        Vector3 sub_displacement   = horizontal_displacement - free_displacement;
        sub_displacement           = sub_displacement.project(surface_normal);
        Vector3 sub_direction      = sub_displacement.normalisedCopy();
        world_transform.m_position = final_position + side_pass_displacement;

        std::vector<PhysicsHitInfo> sub_hits;
        // sweep again for corner case, we only used boolean result in case of jitters
        if (!physics_scene->sweep(
	        m_rigidbody_shape, world_transform.getMatrix(), sub_direction, sub_displacement.length(), sub_hits))
        {
	        side_pass_displacement += sub_displacement;
        }
        final_position += side_pass_displacement;
        return final_position;
    }

} // namespace Pilot
