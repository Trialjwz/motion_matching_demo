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


#include "AppWorldLogic.h"

// World logic, it takes effect only when the world is loaded.
// These methods are called right after corresponding world script's (UnigineScript) methods.

int AppWorldLogic::init()
{
	// convert animations to auxiliary format
	for (int i = ANIM_IDLE; i < NUM_ANIMS; i++)
	{
		anims[i].loadBones(TARGET_MESH_PATH);
		anims[i].loadFrames(source_animations[i]);
	}

	// create dance card
	dance_card.loadBones(TARGET_MESH_PATH);

	dance_card.merge(anims[ANIM_IDLE], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_START_WALKING], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING], ROOT_BONE_NAME);

	dance_card.merge(anims[ANIM_WALKING_LEFT_TURN], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING_LEFT_TURN], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING], ROOT_BONE_NAME);

	dance_card.merge(anims[ANIM_WALKING_LEFT_TURN_FAST], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING_LEFT_TURN_FAST], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING], ROOT_BONE_NAME);

	dance_card.merge(anims[ANIM_WALKING_LEFT_TURN], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING_LEFT_TURN_FAST], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING], ROOT_BONE_NAME);

	dance_card.merge(anims[ANIM_WALKING_RIGHT_TURN], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING_RIGHT_TURN], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING], ROOT_BONE_NAME);

	dance_card.merge(anims[ANIM_WALKING_RIGHT_TURN_FAST], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING_RIGHT_TURN_FAST], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING], ROOT_BONE_NAME);

	dance_card.merge(anims[ANIM_WALKING_RIGHT_TURN], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING_RIGHT_TURN_FAST], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING], ROOT_BONE_NAME);

	dance_card.merge(anims[ANIM_WALKING_LEFT_TURN_180], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING], ROOT_BONE_NAME);

	dance_card.merge(anims[ANIM_WALKING_RIGHT_TURN_180], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_WALKING], ROOT_BONE_NAME);

	dance_card.merge(anims[ANIM_STOP_WALKING], ROOT_BONE_NAME);
	dance_card.merge(anims[ANIM_IDLE], ROOT_BONE_NAME);

	// create simulation bone
	dance_card.generateSimulationBone(SPINE_BONE_NAME, ROOT_BONE_NAME);

	// get velocities and trajectory for each frame
	dance_card.calculateVelocities();
	dance_card.calculateTrajectories(TRAJECTORY_PREDICT_STEP, TRAJECTORY_POINT_NUM);

	// create database
	database.createRows(dance_card);

	// initialize all
	dance_card.setFrame(0);

	inertializer.initialize(dance_card.frames[0], dance_card.frames[0]);

	traj_rotation_springs.resize(TRAJECTORY_POINT_NUM);
	traj_movement_springs.resize(TRAJECTORY_POINT_NUM);

	if (Input::getNumGamePads() > 0)
		game_pad = Input::getGamePad(0);

	bot = checked_ptr_cast<ObjectMeshSkinned>(World::getNodeByName("bot"));

	Visualizer::setEnabled(true);

	return 1;
}

////////////////////////////////////////////////////////////////////////////////
// start of the main loop
////////////////////////////////////////////////////////////////////////////////

int AppWorldLogic::update()
{
	const float ifps = Game::getIFps();

	// update input axis
	if (game_pad.isValid() && game_pad->isAvailable())
	{
		input_axis = Vec3(game_pad->getAxesLeft(), 0.0f);
		if (input_axis.length() < 0.3f)
			input_axis = Vec3_zero;
	} else
	{
		if (Input::isKeyPressed(Input::KEY_T))
			input_axis += Vec3_forward * ifps * 2.0f;
		else if (Input::isKeyPressed(Input::KEY_G))
			input_axis += Vec3_back * ifps * 2.0f;
		else
			input_axis.y *= Math::exp(-ifps * 2.0f);

		if (Input::isKeyPressed(Input::KEY_H))
			input_axis += Vec3_right * ifps * 2.0f;
		else if (Input::isKeyPressed(Input::KEY_F))
			input_axis += Vec3_left * ifps * 2.0f;
		else
			input_axis.x *= Math::exp(-ifps * 2.0f);
	}

	if (input_axis.length() > 1.0f)
		input_axis.normalize();

	// update target velocity
	bot_target_velocity = input_axis * BOT_SPEED;

	// update target rotation
	if (input_axis.length() > 0.0f)
	{
		Vec3 axis(input_axis);
		axis.normalize();
		float angle = -Math::atan2(axis.x, axis.y) * Consts::RAD2DEG;
		bot_target_rotation = quat(vec3_up, angle);
	}

	// update object parameters
	bot_movement_spring.update(bot_target_velocity, MOVEMENT_HALFLIFE, ifps);
	bot_rotation_spring.update(bot_target_rotation, ROTATION_HALFLIFE, ifps);

	Mat4 bot_transform;
	composeTransform(bot_transform, bot_movement_spring.getPosition(), bot_rotation_spring.getRotation(), Vec3_one);
	Mat4 bot_itransform = inverse(bot_transform);

	// predict trajectory rotations
	for (int i = 0; i < TRAJECTORY_POINT_NUM; i++)
	{
		traj_rotation_springs[i].initializeRotation(bot_rotation_spring.getRotation());
		traj_rotation_springs[i].initializeAngularVelocity(bot_rotation_spring.getAngularVelocity());
		traj_rotation_springs[i].update(bot_target_rotation, ROTATION_HALFLIFE, i * TRAJECTORY_PREDICT_STEP);
	}

	// predict trajectory movements
	traj_movement_springs[0].initializePosition(bot_movement_spring.getPosition());
	traj_movement_springs[0].initializeVelocity(bot_movement_spring.getVelocity());
	traj_movement_springs[0].initializeAcceleration(bot_movement_spring.getAcceleration());

	for (int i = 1; i < TRAJECTORY_POINT_NUM; i++)
	{
		traj_movement_springs[i].initializePosition(traj_movement_springs[i - 1].getPosition());
		traj_movement_springs[i].initializeVelocity(traj_movement_springs[i - 1].getVelocity());
		traj_movement_springs[i].initializeAcceleration(traj_movement_springs[i - 1].getAcceleration());
		traj_movement_springs[i].update(bot_target_velocity, MOVEMENT_HALFLIFE, TRAJECTORY_PREDICT_STEP);
	}

	// frame tick
	frame_timer += ifps;
	motion_matching_timer += ifps;
	if (frame_timer >= (1.0f / 30))
	{
		frame_timer = 0.0f;

		int frame = dance_card.current_frame_index;
		int old_frame = frame;

		// motion matching
		if (motion_matching_timer > 0.25f)
		{
			motion_matching_timer = 0.0f;

			DataBase::Row query_row;
			query_row = database.rows[frame];

			for (int i = 0; i < TRAJECTORY_POINT_NUM; i++)
			{
				query_row.traj_positions[i] = bot_itransform * traj_movement_springs[i].getPosition();
				query_row.traj_directions[i] = bot_itransform.getRotate() * (traj_rotation_springs[i].getRotation() * Vec3_forward);
			}

			int best_frame = database.find(query_row);
			if (best_frame != -1)
				frame = best_frame;
		}

		frame++;
		if (frame >= dance_card.frames.size())
			frame = 0;

		// new transition
		inertializer.initialize(dance_card.frames[old_frame], dance_card.frames[frame]);

		dance_card.setFrame(frame);
	}

	// update transition
	inertializer.update(dance_card.frames[dance_card.current_frame_index], 0.1f, ifps);

	Frame current_frame = dance_card.frames[dance_card.current_frame_index];

	// update simulation bone...
	current_frame.transforms.last().local = bot_transform;

	// ...and other bones
	for (int i = 0; i < dance_card.bones.size() - 1; i++)
	{
		current_frame.transforms[i].local_pos = inertializer.getPosition(i);
		current_frame.transforms[i].local_rot = inertializer.getRotation(i);
		current_frame.transforms[i].composeLocal();
	}

	current_frame.updateWorldTransforms(dance_card.bones, dance_card.roots);
	dance_card.current_frame = current_frame;

	// apply bones
	if (bot.isValid())
	{
		bot->setWorldTransform(bot_transform);
		for (int i = 0; i < dance_card.bones.size() - 1; i++)
			bot->setBoneTransform(i, bot_itransform * dance_card.current_frame.transforms[i].world);
	}

	// render object
	Visualizer::renderCircle(0.5f, translate(bot_movement_spring.getPosition()), vec4_blue);
	Visualizer::renderVector(bot_movement_spring.getPosition(),
							 bot_movement_spring.getPosition() + bot_rotation_spring.getRotation() * Vec3_forward * 0.6f,
							 vec4_blue);

	// render trajectory
	for (int i = 0; i < TRAJECTORY_POINT_NUM; i++)
	{
		Visualizer::renderSphere(0.05f, translate(traj_movement_springs[i].getPosition()), vec4_red);

		Visualizer::renderVector(traj_movement_springs[i].getPosition(),
								 traj_movement_springs[i].getPosition() + traj_rotation_springs[i].getRotation() * Vec3_forward * 0.3f,
								 vec4_red);
	}

	database.render(dance_card);

	return 1;
}
