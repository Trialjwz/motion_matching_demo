/* Copyright (C) 2005-2023, UNIGINE. All rights reserved.
 *
 * This file is a part of the UNIGINE 2 SDK.
 *
 * Your use and / or redistribution of this software in source and / or
 * binary form, with or without modification, is subject to: (i) your
 * ongoing acceptance of and compliance with the terms and conditions of
 * the UNIGINE License Agreement; and (ii) your inclusion of this notice
 * in any version of this software that you use or redistribute.
 * A copy of the UNIGINE License Agreement is available by contacting
 * UNIGINE. at http://unigine.com/
 */


#ifndef __APP_WORLD_LOGIC_H__
#define __APP_WORLD_LOGIC_H__

#include <UnigineLogic.h>
#include <UnigineStreams.h>
#include <Unigine.h>
#include "MotionMatching.h"

using namespace Unigine;

class AppWorldLogic final : public Unigine::WorldLogic
{

public:
	int init() override;
	int update() override;

private:

	// all available animations
	enum ANIM
	{
		ANIM_IDLE = 0,
		ANIM_START_WALKING,
		ANIM_WALKING,
		ANIM_STOP_WALKING,
		ANIM_WALKING_LEFT_TURN,
		ANIM_WALKING_LEFT_TURN_FAST,
		ANIM_WALKING_LEFT_TURN_180,
		ANIM_WALKING_RIGHT_TURN,
		ANIM_WALKING_RIGHT_TURN_FAST,
		ANIM_WALKING_RIGHT_TURN_180,
		NUM_ANIMS,
	};

	const HashMap<int, String> source_animations = {
		{ANIM_IDLE,                    "animations/idle.anim"},
		{ANIM_START_WALKING,           "animations/start_walking.anim"},
		{ANIM_WALKING,                 "animations/walking.anim"},
		{ANIM_STOP_WALKING,            "animations/stop_walking.anim"},
		{ANIM_WALKING_LEFT_TURN,       "animations/walking_left_turn.anim"},
		{ANIM_WALKING_LEFT_TURN_FAST,  "animations/walking_left_turn_fast.anim"},
		{ANIM_WALKING_RIGHT_TURN,      "animations/walking_right_turn.anim"},
		{ANIM_WALKING_RIGHT_TURN_FAST, "animations/walking_right_turn_fast.anim"},
		{ANIM_WALKING_LEFT_TURN_180,   "animations/walking_left_turn_180.anim"},
		{ANIM_WALKING_RIGHT_TURN_180,  "animations/walking_right_turn_180.anim"},
	};

	// auxiliary animation objects
	const char *TARGET_MESH_PATH = "Y Bot.fbx/alpha_surface.mesh";
	const char *ROOT_BONE_NAME = "mixamorig:Hips";
	const char *SPINE_BONE_NAME = "mixamorig:Spine1";

	HashMap<int, Animation> anims = {
		{ANIM_IDLE, {}},
		{ANIM_START_WALKING, {}},
		{ANIM_WALKING, {}},
		{ANIM_STOP_WALKING, {}},
		{ANIM_WALKING_LEFT_TURN, {}},
		{ANIM_WALKING_LEFT_TURN_FAST, {}},
		{ANIM_WALKING_RIGHT_TURN, {}},
		{ANIM_WALKING_RIGHT_TURN_FAST, {}},
		{ANIM_WALKING_LEFT_TURN_180, {}},
		{ANIM_WALKING_RIGHT_TURN_180, {}},
	};

	Animation dance_card;
	DataBase database;

	// input
	Vec3 input_axis;
	InputGamePadPtr game_pad;

	// bot movement
	ObjectMeshSkinnedPtr bot;
	Vec3 bot_target_velocity{Vec3_one};
	quat bot_target_rotation{quat_identity};

	const float BOT_SPEED = 1.51f;
	const float ROTATION_HALFLIFE = 0.3f;
	const float MOVEMENT_HALFLIFE = 0.4f;

	SpringMovement bot_movement_spring;
	SpringRotation bot_rotation_spring;

	Inertializer inertializer;

	// bot trajectory
	const int TRAJECTORY_POINT_NUM = 5;
	const float TRAJECTORY_PREDICT_STEP = 0.25f;

	Vector<SpringRotation> traj_rotation_springs;
	Vector<SpringMovement> traj_movement_springs;

	// timers
	float frame_timer{0.0f};
	float motion_matching_timer{0.0f};
};

#endif // __APP_WORLD_LOGIC_H__
