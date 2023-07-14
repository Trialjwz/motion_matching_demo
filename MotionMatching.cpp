#include "MotionMatching.h"
#include <UnigineVisualizer.h>

///////////////////////////////////////////////////////
// Animation
///////////////////////////////////////////////////////

void Frame::updateWorldTransforms(const Vector<Bone> &bones, const Vector<int> &roots)
{
	for (int r : roots)
		update_world_transforms(r, bones);
}

void Frame::update_world_transforms(int bone, const Vector<Bone> &bones)
{
	int parent = bones[bone].parent;
	if (parent == -1)
		transforms[bone].world = transforms[bone].local;
	else
		transforms[bone].world = transforms[parent].world * transforms[bone].local;

	transforms[bone].decomposeWorld();

	for (int c : bones[bone].children)
		update_world_transforms(c, bones);
}

void Frame::renderBones(const Vector<Bone> &bones, const Vector<int> &roots)
{
	for (int r : roots)
		render_bones(r, bones);

	// simulation bone
	const Mat4 &t = transforms.last().world;
	Visualizer::renderVector(t.getTranslate(), t.getTranslate() + t.getAxisY(), vec4_red);
}

void Frame::render_bones(int bone, const Vector<Bone> &bones)
{
	int parent = bones[bone].parent;
	if (parent != -1)
	{
		Vec3 p0 = transforms[bone].world_pos;
		Vec3 p1 = transforms[parent].world_pos;
		Visualizer::renderLine3D(p0, p1, vec4_red);
	}

	Visualizer::renderSolidSphere(0.005f, transforms[bone].world, vec4_red);

	for (int c : bones[bone].children)
		render_bones(c, bones);
}

void Frame::renderVelocities()
{
	for (int i = 0; i < transforms.size(); i++)
	{
		Visualizer::renderVector(transforms[i].world_pos,
								 transforms[i].world_pos + velocities[i].world_velocity,
								 vec4_green);
	}
}

void Frame::renderTrajectory()
{
	for (const Mat4 &t : trajectory_points)
	{
		Visualizer::renderSolidSphere(0.03f, t, vec4_blue);
		Visualizer::renderVector(t.getTranslate(),
								 t.getTranslate() + t.getAxisY() * 0.3f,
								 vec4_blue);
	}
}

void Animation::loadBones(const char *target_mesh_path)
{
	MeshPtr mesh = Mesh::create(target_mesh_path);

	int num_bones = mesh->getNumBones();
	bones.resize(num_bones);
	for (int i = 0; i < num_bones; i++)
	{
		bones[i].name = mesh->getBoneName(i);
		bones[i].parent = mesh->getBoneParent(i);

		if (bones[i].parent != -1)
			bones[bones[i].parent].children.append(i);
		else
			roots.append(i);
	}
}

void Animation::loadFrames(const char *target_anim_path)
{
	MeshPtr anim_mesh = Mesh::create(target_anim_path);

	int num_frames = anim_mesh->getNumAnimationFrames(0);
	frames.resize(num_frames);
	for (int i = 0; i < num_frames; i++)
	{
		frames[i].transforms.resize(bones.size());

		Vector<mat4> values;
		anim_mesh->getAnimationFrame(0, i, values);

		for (int j = 0; j < bones.size(); j++)
		{
			// remap bone by name
			int bone_index = anim_mesh->findBone(bones[j].name);
			if (bone_index == -1)
				continue;

			frames[i].transforms[j].local = values[bone_index];
			frames[i].transforms[j].decomposeLocal();
		}
	}

	for (auto &f : frames)
		f.updateWorldTransforms(bones, roots);

	setFrame(0);
}

void Animation::merge(const Animation &anim, const char *root_bone_name)
{
	if (frames.empty())
	{
		frames = anim.frames;
		return;
	}

	int root_index = -1;
	for (int i = 0; i < bones.size(); i++)
	{
		if (strcmp(bones[i].name, root_bone_name) == 0)
		{
			root_index = i;
			break;
		}
	}

	if (root_index == -1)
		return;

	// get root offset
	Mat4 root_transform = frames.last().transforms[root_index].world;
	Vec3 proj_pos = projectOntoPlane(root_transform.getTranslate(), Vec3_up);
	Vec3 proj_dir_pos = projectOntoPlane(root_transform.getTranslate() + root_transform.getAxisY(), Vec3_up);
	Vec3 dir = (proj_dir_pos - proj_pos).normalize();
	float dir_angle = -Math::atan2(dir.x, dir.y) * Consts::RAD2DEG;

	Mat4 root_offset;
	composeTransform(root_offset, proj_pos, quat(vec3_up, dir_angle), vec3_one);

	// add new frames with offset
	for (int i = 0; i < anim.frames.size(); i++)
	{
		Frame &f = frames.append();
		f = anim.frames[i];

		for (int r : roots)
		{
			f.transforms[r].local = root_offset * f.transforms[r].local;
			f.transforms[r].decomposeLocal();
		}
		f.updateWorldTransforms(bones, roots);
	}
}

void Animation::generateSimulationBone(const char *pos_bone_name, const char *rot_bone_name)
{
	int pos_bone_index = -1;
	int rot_bone_index = -1;

	for (int i = 0; i < bones.size(); i++)
	{
		if (pos_bone_index == -1 && strcmp(bones[i].name, pos_bone_name) == 0)
			pos_bone_index = i;

		if (rot_bone_index == -1 && strcmp(bones[i].name, rot_bone_name) == 0)
			rot_bone_index = i;

		if (pos_bone_index != -1 && rot_bone_index != -1)
			break;
	}

	if (pos_bone_index == -1 || rot_bone_index == -1)
		return;

	// get projected positions and directions...
	Vector<Vec3> positions;
	Vector<Vec3> directions;
	for (int i = 0; i < frames.size(); i++)
	{
		Vec3 pos = projectOntoPlane(frames[i].transforms[pos_bone_index].world_pos, Vec3_up);
		positions.append(pos);

		Mat4 rot_bone_transform = frames[i].transforms[rot_bone_index].world;
		Vec3 rot_proj_pos = projectOntoPlane(rot_bone_transform.getTranslate(), Vec3_up);
		Vec3 rot_proj_dir_pos = projectOntoPlane(rot_bone_transform.getTranslate() + rot_bone_transform.getAxisY(), Vec3_up);
		Vec3 dir = (rot_proj_dir_pos - rot_proj_pos).normalize();

		directions.append(dir);
	}

	// ... smooth it ...
	savitzky_golay_filter(positions);
	savitzky_golay_filter(directions);

	for (int i = 0; i < frames.size(); i++)
		directions[i].normalize();

	// ... and create transformations for new bone
	Vector<Mat4> transforms;
	for (int i = 0; i < frames.size(); i++)
	{
		Mat4 &t = transforms.append();
		composeTransform(t, positions[i], quat(vec3_up, -Math::atan2(directions[i].x, directions[i].y) * Consts::RAD2DEG), vec3_one);
	}

	// add new bone transformations and change current roots
	for (int i = 0; i < frames.size(); i++)
	{
		BoneTransform &bt = frames[i].transforms.append();
		bt.local = transforms[i];

		for (int r : roots)
		{
			frames[i].transforms[r].local = inverse(bt.local) * frames[i].transforms[r].local;
			frames[i].transforms[r].decomposeLocal();
		}
	}

	// add new bone to hierarchy
	Bone &sim_bone = bones.append();
	sim_bone.name = "sim_bone";
	sim_bone.parent = -1;
	for (int r : roots)
	{
		bones[r].parent = bones.size() - 1;
		sim_bone.children.append(r);
	}
	roots.clear();
	roots.append(bones.size() - 1);

	for (auto &f : frames)
		f.updateWorldTransforms(bones, roots);
}

void Animation::calculateVelocities()
{
	const float dt = 1 / 30.0f;

	for (int i = 0; i < frames.size(); i++)
	{
		Frame &frame = frames[i];
		frame.velocities.resize(bones.size());

		if (i == 0)
		{
			// forward difference
			const Frame &next_frame = frames[i + 1];
			for (int j = 0; j < bones.size(); j++)
			{
				frame.velocities[j].local_velocity = (next_frame.transforms[j].local_pos - frame.transforms[j].local_pos) / dt;

				quat delta_rot = next_frame.transforms[j].local_rot * inverse(frame.transforms[j].local_rot);
				delta_rot = abs(delta_rot);
				frame.velocities[j].local_angular_velocity = (log(delta_rot) * 2.0f) / dt;
			}
		} else if (i == frames.size() - 1)
		{
			// backward difference
			const Frame &prev_frame = frames[i - 1];
			for (int j = 0; j < bones.size(); j++)
			{
				frame.velocities[j].local_velocity = (frame.transforms[j].local_pos - prev_frame.transforms[j].local_pos) / dt;

				quat delta_rot = prev_frame.transforms[j].local_rot * inverse(frame.transforms[j].local_rot);
				delta_rot = abs(delta_rot);
				frame.velocities[j].local_angular_velocity = (log(delta_rot) * 2.0f) / dt;
			}
		} else
		{
			// central difference
			const Frame &next_frame = frames[i + 1];
			const Frame &prev_frame = frames[i - 1];
			for (int j = 0; j < bones.size(); j++)
			{
				frame.velocities[j].local_velocity = (next_frame.transforms[j].local_pos - prev_frame.transforms[j].local_pos) / (2.0f * dt);

				quat delta_rot = next_frame.transforms[j].local_rot * inverse(prev_frame.transforms[j].local_rot);
				delta_rot = abs(delta_rot);
				frame.velocities[j].local_angular_velocity = (log(delta_rot) * 2.0f) / (2.0f * dt);
			}
		}

		for (int root : roots)
			update_world_velocities(root, i);
	}
}

void Animation::update_world_velocities(int bone_index, int frame_index)
{
	const Bone &bone = bones[bone_index];
	Frame &frame = frames[frame_index];

	if (bone.parent == -1)
	{
		frame.velocities[bone_index].world_velocity = frame.velocities[bone_index].local_velocity;
		frame.velocities[bone_index].world_angular_velocity = frame.velocities[bone_index].local_angular_velocity;
	} else
	{
		int p = bone.parent;

		frame.velocities[bone_index].world_velocity = frame.velocities[p].world_velocity +
			frame.transforms[p].world_rot * frame.velocities[bone_index].local_velocity +
			cross(frame.velocities[p].world_angular_velocity, frame.transforms[p].world_rot * frame.transforms[bone_index].local_pos);

		frame.velocities[bone_index].world_angular_velocity = frame.velocities[p].world_angular_velocity +
			frame.transforms[p].world_rot * frame.velocities[bone_index].local_angular_velocity;
	}

	for (int c : bone.children)
		update_world_velocities(c, frame_index);
}

void Animation::calculateTrajectories(float interval, int num_points)
{
	// NOTICE: last bone is simulation bone
	const float frame_step = 1.0f / 30.0f;
	const int max_offset = ftoi((interval * (num_points - 1)) / frame_step);

	for (int i = 0; i < num_points; i++)
	{
		float time = interval * i;
		int frame_offset = ftoi(time / frame_step);

		for (int j = 0; j < frames.size() - max_offset; j++)
			frames[j].trajectory_points.append(frames[j + frame_offset].transforms.last().world);
	}

	for (int i = frames.size() - max_offset; i < frames.size(); i++)
	{
		for (int j = 0; j < num_points; j++)
			frames[i].trajectory_points.append(frames[i].transforms.last().world);
	}
}

void Animation::setFrame(int frame)
{
	current_frame_index = clamp(frame, 0, frames.size() - 1);
	current_frame = frames[current_frame_index];
}

///////////////////////////////////////////////////////
// Database
///////////////////////////////////////////////////////

void DataBase::createRows(const Animation &anim)
{
	// NOTICE: last bone is simulation bone

	rows.resize(anim.frames.size());
	for (int i = 0; i < anim.frames.size(); i++)
	{
		const Frame &f = anim.frames[i];
		Row &r = rows[i];

		Mat4 t = inverse(f.transforms.last().world);
		quat rot = t.getRotate();

		r.left_foot_position = t * f.transforms[L_FOOT].world_pos;
		r.right_foot_position = t * f.transforms[R_FOOT].world_pos;

		r.left_foot_velocity = rot * f.velocities[L_FOOT].world_velocity;
		r.right_foot_velocity = rot * f.velocities[R_FOOT].world_velocity;
		r.hips_velocity = rot * f.velocities[HIPS].world_velocity;

		for (int i = 0; i < f.trajectory_points.size(); i++)
		{
			r.traj_positions.append(t * f.trajectory_points[i].getTranslate());
			r.traj_directions.append(rot * f.trajectory_points[i].getAxisY());
		}
	}
}

int DataBase::find(const Row &query) const
{
	float min_cost = Consts::INF;
	int best_frame = -1;

	for (int i = 0; i < rows.size(); i++)
	{
		// find delta
		Row delta;
		delta.left_foot_position = rows[i].left_foot_position - query.left_foot_position;
		delta.right_foot_position = rows[i].right_foot_position - query.right_foot_position;
		delta.left_foot_velocity = rows[i].left_foot_velocity - query.left_foot_velocity;
		delta.right_foot_velocity = rows[i].right_foot_velocity - query.right_foot_velocity;
		delta.hips_velocity = rows[i].hips_velocity - query.hips_velocity;

		for (int j = 0; j < query.traj_positions.size(); j++)
		{
			delta.traj_positions.append(rows[i].traj_positions[j] - query.traj_positions[j]);
			delta.traj_directions.append(rows[i].traj_directions[j] - query.traj_directions[j]);
		}

		// get new cost
		float cost = 0.0f;
		cost += delta.left_foot_position.length2();
		cost += delta.right_foot_position.length2();
		cost += delta.left_foot_velocity.length2();
		cost += delta.right_foot_velocity.length2();
		cost += delta.hips_velocity.length2();

		for (int j = 0; j < delta.traj_positions.size(); j++)
		{
			cost += delta.traj_positions[j].length2();
			cost += delta.traj_directions[j].length2();
		}

		if (cost < min_cost)
		{
			min_cost = cost;
			best_frame = i;
		}
	}

	return best_frame;
}

void DataBase::render(const Animation &anim)
{
	// NOTICE: last bone is simulation bone

	int index = anim.current_frame_index;
	const Row &r = rows[index];

	Mat4 t = anim.current_frame.transforms.last().world;
	quat rot = t.getRotate();

	Visualizer::renderSolidSphere(0.005f, translate(t * r.left_foot_position), vec4_green);
	Visualizer::renderSolidSphere(0.005f, translate(t * r.right_foot_position), vec4_green);

	Visualizer::renderVector(t * r.left_foot_position, t * r.left_foot_position + rot * r.left_foot_velocity, vec4_green);
	Visualizer::renderVector(t * r.right_foot_position, t * r.right_foot_position + rot * r.right_foot_velocity, vec4_green);

	for (int i = 0; i < r.traj_positions.size(); i++)
	{
		Visualizer::renderSolidSphere(0.005f, translate(t * r.traj_positions[i]), vec4_green);
		Visualizer::renderVector(t * r.traj_positions[i], t * r.traj_positions[i] + rot * r.traj_directions[i] * 0.3f, vec4_green);
	}
}
