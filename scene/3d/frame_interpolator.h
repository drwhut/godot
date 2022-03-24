/*************************************************************************/
/*  frame_interpolator.h                                                 */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2022 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2022 Godot Engine contributors (cf. AUTHORS.md).   */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

// This work has been kindly sponsored by IMVU.
// Version 3.1: faster and better recovering algorithm, more optimized, supports a dynamic physics frame.

/** @author: AndreaCatania */

#ifndef FRAME_INTERPOLATOR_H
#define FRAME_INTERPOLATOR_H

#include "core/local_vector.h"
#include "core/math/math_defs.h"
#include "core/math/math_funcs.h"
#include "core/object.h"
#include "core/print_string.h"

// Used to debug the interpolation.
//#define DEBUG_FRAME_INTERPOLATOR

template <class T>
class FrameInterpolator {

	T next_data;
	T current_data;
	real_t time_ahead = 0.0;

	real_t speed_factor = 1.0;
	real_t last_insert_delta_time = 0.0;
	real_t last_remaining_time_at_insert = 0.0;

	/// Defines how many dynamic frames the interpolation is behind.
	/// The speedup mechanism will keep the interpolation behind by:
	/// (ideal_frames_ahead * delta) + physics_delta
	real_t ideal_frames_ahead = 3.0;
	/// The bounds for the speed_factor, default 20% of delta.
	real_t speed_factor_bounds = 0.2;
	/// The sigmoid factor can be used to change the sigmoid shape so that
	/// the speed of the recovery is also changed.
	/// With a factor of 1.0 the mechanism will try to recover the change in 1.0
	/// frame. A too high factor can make the mechanism too hard and the recoverage
	/// will be too noticeable.
	real_t sigmoid_factor = 0.2;
	/// The rate of change at witch the interpolation speed changes.
	/// This allow to smooth the interpolation speed so to remove the noise
	/// caused by the  dynamic delta.
	/// The speedup per frame: `lerp(speed_factor, desired_speed_factor, speedup * p_delta)`
	real_t speedup = 10.0;
	/// How many physical frames it's allowed to stay behind.
	/// This is useful to fastforward the interpolator to more up-to-date
	/// information.
	/// This happens when you have a really long frame and many physics frames
	/// are processed in the same frame.
	uint32_t fallback_amount = 10;

public:
	/// Reset the frame interpolation. Call this whenever the data store are
	/// outdated.
	void reset(const T &p_current);

	/// Push the next information to interpolate. Must be called each
	/// physics_process.
	void push_data(const T &p_data, real_t p_delta);

	/// Returns the interpolated data. This must be called each `process`.
	T get_next_frame_data(real_t p_delta);
};

template <class T>
void FrameInterpolator<T>::reset(
		const T &p_current) {

	next_data = p_current;
	current_data = p_current;
	time_ahead = 0.0;
	last_insert_delta_time = 0.0;
	last_remaining_time_at_insert = 0.0;
}

template <class T>
void FrameInterpolator<T>::push_data(const T &p_data, real_t p_delta) {
	next_data = p_data;
	last_insert_delta_time = p_delta;
	last_remaining_time_at_insert = time_ahead;
#ifdef DEBUG_FRAME_INTERPOLATOR
	print_line("Time ahead: " + rtos(time_ahead) + ", Delta: " + rtos(p_delta));
#endif
	time_ahead += p_delta;
}

template <class T>
T FrameInterpolator<T>::get_next_frame_data(real_t p_delta) {
	// TODO this is an hack because sometimes the engine gives 0 delta when the
	// physics_iteration is changed.
	p_delta = MAX(p_delta, CMP_EPSILON);

	if (time_ahead > 0.0) {

		if (unlikely(time_ahead > (real_t(fallback_amount) * last_insert_delta_time))) {
			// Move the timeline forward till the 20% fallback.
			p_delta = time_ahead - (real_t(fallback_amount) * 0.2 * last_insert_delta_time);
		} else {
			// Computes the speed_factor
			// The `ideal` is used to know the amount of time the interpolator
			// has to stay behind the new received frame.
			const real_t ideal = p_delta * ideal_frames_ahead;
			// The `delta_ideal` represents the distance to the `ideal` time.
			// It's used to determine the speedup amount and direction.
			const real_t delta_ideal = last_remaining_time_at_insert - ideal;
			// The `delta_ideal` is feed into the sigmoid function `tanh` that
			// returns the value compressed in the range of -1 / 1.
			const real_t sigmoid = Math::tanh((delta_ideal / p_delta) * sigmoid_factor);
			// The `sigmoid` is now converted to the `desired_speed_factor` for
			// this frame.
			const real_t desired_speed_factor = 1.0 + sigmoid * speed_factor_bounds;
			// The `speed_factor` is interpolated to the `desired_speed_factor`
			// so that we smoothly transition to the new value.
			// In this way we can remove the noise of the `p_delta`.
			speed_factor = Math::lerp(speed_factor, desired_speed_factor, speedup * p_delta);
			speed_factor = CLAMP(speed_factor, 1.0 - speed_factor_bounds, 1.0 + speed_factor_bounds);
#ifdef DEBUG_FRAME_INTERPOLATOR
			print_line("Speed factor: " + rtos(speed_factor) + " ~~ Desired speed factor: " + rtos(desired_speed_factor) + " ~~ Ideal frame: " + rtos(ideal) + " ~~ Delta ideal: " + rtos(delta_ideal) + " ~~ Sigmoid: " + rtos(sigmoid) + " ~~ Time ahead: " + rtos(time_ahead));
#endif
		}

		// Advance the time.
		const real_t adjusted_delta = MIN(p_delta * speed_factor, time_ahead);
		const real_t interpolation_factor = adjusted_delta / time_ahead;
		time_ahead -= adjusted_delta;

		current_data = current_data.interpolate_with(next_data, interpolation_factor);
	} else {
		// Computes the speed_factor
		speed_factor -= speedup * p_delta;
		speed_factor = MAX(speed_factor, 1.0 - speed_factor_bounds);
	}

	return current_data;
}

#endif
