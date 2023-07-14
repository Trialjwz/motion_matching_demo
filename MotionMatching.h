#pragma once
#include <UnigineMathLib.h>
#include <UnigineVector.h>

using namespace Unigine;
using namespace Math;

///////////////////////////////////////////////////////
// Utils
///////////////////////////////////////////////////////

static void savitzky_golay_filter(Vector<Vec3> &points)
{
	if (points.size() < 7)
		return;

	Vector<Vec3> filtered_points(points);
	for (int i = 3; i < points.size() - 3; i++)
	{
		filtered_points[i] = (-points[i - 3] * 2.0f + points[i - 2] * 3.0f + points[i - 1] * 6.0f + points[i] * 7.0f +
							  points[i + 1] * 6.0f + points[i + 2] * 3.0f - points[i + 3] * 2.0f) / 21.0f;
	}
	points = std::move(filtered_points);
}

static Vec3 log(const quat &q)
{
	float length = Math::fsqrt(q.x * q.x + q.y * q.y + q.z * q.z);

	if (length < Consts::EPS)
		return Vec3(q.x, q.y, q.z);

	float halfangle = Math::acos(clamp(q.w, -1.0f, 1.0f));
	return (Vec3(q.x, q.y, q.z) / length) * halfangle;
}

static quat exp(const Vec3 &v)
{
	float halfangle = Math::fsqrt(v.x * v.x + v.y * v.y + v.z * v.z);

	if (halfangle < Consts::EPS)
		return normalize(quat(vec4(v.x, v.y, v.z, 1.0f)));

	float c = Math::cos(halfangle);
	float s = Math::sin(halfangle) / halfangle;
	return quat(vec4(s * v.x, s * v.y, s * v.z, c));
}

static quat abs(const quat &x)
{
	return x.w < 0.0 ? -x : x;
}

///////////////////////////////////////////////////////
// Bone
///////////////////////////////////////////////////////

struct BoneTransform
{
	vec3 local_pos;
	quat local_rot;
	vec3 local_scale{vec3_one};
	mat4 local;

	Vec3 world_pos;
	quat world_rot;
	vec3 world_scale{vec3_one};
	Mat4 world;

	void composeLocal() { composeTransform(local, local_pos, local_rot, local_scale); }
	void decomposeLocal() { decomposeTransform(local, local_pos, local_rot, local_scale); }

	void composeWorld() { composeTransform(world, world_pos, world_rot, world_scale); }
	void decomposeWorld() { decomposeTransform(world, world_pos, world_rot, world_scale); }
};

struct BoneVelocities
{
	vec3 local_velocity;
	vec3 local_angular_velocity;

	vec3 world_velocity;
	vec3 world_angular_velocity;
};

struct Bone
{
	String name;
	int parent{-1};
	Vector<int> children;
};

///////////////////////////////////////////////////////
// Animation
///////////////////////////////////////////////////////

struct Frame
{
	Vector<BoneTransform> transforms;
	Vector<BoneVelocities> velocities;
	Vector<Mat4> trajectory_points;

	void updateWorldTransforms(const Vector<Bone> &bones, const Vector<int> &roots);
	void update_world_transforms(int bone, const Vector<Bone> &bones);

	void renderBones(const Vector<Bone> &bones, const Vector<int> &roots);
	void render_bones(int bone, const Vector<Bone> &bones);

	void renderVelocities();
	void renderTrajectory();
};

struct Animation
{
	Vector<Bone> bones;
	Vector<int> roots;

	Vector<Frame> frames;
	Frame current_frame;
	int current_frame_index{-1};

	void loadBones(const char *target_mesh_path);
	void loadFrames(const char *target_anim_path);

	void merge(const Animation &anim, const char *root_bone_name);

	void generateSimulationBone(const char *pos_bone_name, const char *rot_bone_name);

	void calculateVelocities();
	void update_world_velocities(int bone_index, int frame_index);

	void calculateTrajectories(float interval, int num_points);

	void setFrame(int frame);

	void renderCurrentBones() { current_frame.renderBones(bones, roots); }
};

///////////////////////////////////////////////////////
// Database
///////////////////////////////////////////////////////

struct DataBase
{
	const int L_FOOT = 46;
	const int R_FOOT = 50;
	const int HIPS = 0;

	struct Row
	{
		Vec3 left_foot_position;
		Vec3 right_foot_position;

		Vec3 left_foot_velocity;
		Vec3 right_foot_velocity;
		Vec3 hips_velocity;

		Vector<Vec3> traj_positions;
		Vector<Vec3> traj_directions;
	};

	Vector<Row> rows;

	void createRows(const Animation &anim);
	int find(const Row &query) const;
	void render(const Animation &anim);
};

///////////////////////////////////////////////////////
// Springs
///////////////////////////////////////////////////////

struct SpringMovement
{
	const Vec3 &getPosition() const { return position; }
	const Vec3 &getVelocity() const { return velocity; }
	const Vec3 &getAcceleration() const { return acceleration; }

	void initializePosition(const Vec3 &pos) { position = pos; }
	void initializeVelocity(const Vec3 &v) { velocity = v; }
	void initializeAcceleration(const Vec3 &a) { acceleration = a; }

	void update(const Vec3 &velocity_dst, float halflife, float dt)
	{
		// NOTICE:
		// spring position = character velocity
		// spring velocity = character acceleration

		float y = ((4.0f * Consts::LOG2) / Math::max(halflife, Consts::EPS)) / 2.0f;
		Vec3 j0 = velocity - velocity_dst;
		Vec3 j1 = acceleration + j0 * y;
		float exp_value = Math::exp(-y * dt);

		position = (-j1 / (y * y) + (-j0 - j1 * dt) / y) * exp_value +
			j1 / (y * y) + j0 / y + velocity_dst * dt + position;

		velocity = (j0 + j1 * dt) * exp_value + velocity_dst;
		acceleration = (acceleration - j1 * y * dt) * exp_value;
	}

private:
	Vec3 position;
	Vec3 velocity;
	Vec3 acceleration;
};

struct SpringRotation
{
	const quat &getRotation() const { return rotation; }
	const vec3 &getAngularVelocity() const { return angular_velocity; }

	void initializeRotation(const quat &rot) { rotation = rot; }
	void initializeAngularVelocity(const vec3 &v) { angular_velocity = v; }

	void update(const quat &rotation_dst, float halflife, float dt)
	{
		float y = ((4.0f * Consts::LOG2) / Math::max(halflife, Consts::EPS)) / 2.0f;
		vec3 j0 = log(abs(rotation * inverse(rotation_dst))) * 2.0f;
		vec3 j1 = angular_velocity + j0 * y;
		float exp_value = Math::exp(-y * dt);

		rotation = exp((j0 + j1 * dt) * exp_value * 0.5f) * rotation_dst;
		angular_velocity = (angular_velocity - j1 * y * dt) * exp_value;
	}

private:
	quat rotation;
	vec3 angular_velocity;
};

///////////////////////////////////////////////////////
// Inertialization
///////////////////////////////////////////////////////

struct Inertializer
{
	const vec3 &getPosition(int index) const { return positions[index]; }
	const quat &getRotation(int index) const { return rotations[index]; }

	void initialize(const Frame &src, const Frame &dst)
	{
		int num = src.transforms.size();
		position_offsets.resize(num);
		velocity_offsets.resize(num);
		rotation_offsets.resize(num);
		angular_velocity_offsets.resize(num);

		positions.resize(num);
		rotations.resize(num);

		for (int i = 0; i < num; i++)
		{
			position_offsets[i] += src.transforms[i].local_pos - dst.transforms[i].local_pos;
			velocity_offsets[i] += src.velocities[i].local_velocity - dst.velocities[i].local_velocity;
			rotation_offsets[i] = abs((rotation_offsets[i] * src.transforms[i].local_rot) * inverse(dst.transforms[i].local_rot));
			angular_velocity_offsets[i] += src.velocities[i].local_angular_velocity - dst.velocities[i].local_angular_velocity;
		}
	}

	void update(const Frame &current_frame, float halflife, float dt)
	{
		for (int i = 0; i < current_frame.transforms.size(); i++)
		{
			float y = (4.0f * Consts::LOG2) / Math::max(halflife, Consts::EPS) / 2.0f;
			float exp_value = Math::exp(-y * dt);

			// position offset
			{
				vec3 j1 = velocity_offsets[i] + position_offsets[i] * y;

				position_offsets[i] = (position_offsets[i] + j1 * dt) * exp_value;
				velocity_offsets[i] = (velocity_offsets[i] - j1 * y * dt) * exp_value;
			}

			// rotation offset
			{
				vec3 j0 = log(rotation_offsets[i]) * 2.0f;
				vec3 j1 = angular_velocity_offsets[i] + j0 * y;

				rotation_offsets[i] = exp((j0 + j1 * dt) * exp_value * 0.5f);
				angular_velocity_offsets[i] = (angular_velocity_offsets[i] - j1 * y * dt) * exp_value;
			}

			positions[i] = current_frame.transforms[i].local_pos + position_offsets[i];
			rotations[i] = rotation_offsets[i] * current_frame.transforms[i].local_rot;
		}
	}

private:
	Vector<vec3> position_offsets;
	Vector<vec3> velocity_offsets;
	Vector<quat> rotation_offsets;
	Vector<vec3> angular_velocity_offsets;

	Vector<vec3> positions;
	Vector<quat> rotations;
};
