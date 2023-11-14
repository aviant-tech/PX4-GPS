/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <containers/IntrusiveSortedList.hpp>
#include <uavcan/uavcan.hpp>
#include <lib/parameters/param.h>
#include <drivers/drv_hrt.h>

#include <uavcan/node/publisher.hpp>

namespace uavcannode
{

class UavcanPublisherBase : public IntrusiveSortedListNode<UavcanPublisherBase *>
{
public:
	UavcanPublisherBase() = delete;
	explicit UavcanPublisherBase(uint16_t id) : _id(id)
	{
		int32_t delay_ms;
		param_get(param_find("CANNODE_DELAY_MS"), &delay_ms);

		if (delay_ms < 0) {
			return;
		}

		delay_us = static_cast<hrt_abstime>(delay_ms) * hrt_abstime{1000};
	}

	virtual ~UavcanPublisherBase() = default;

	/**
	 * Prints current status in a human readable format to stdout.
	 */
	virtual void PrintInfo() = 0;

	virtual void BroadcastAnyUpdates() = 0;

	// sorted numerically by ID
	bool operator<=(UavcanPublisherBase &rhs) { return id() <= rhs.id(); }

	uint16_t id() const { return _id; }

	bool readyToPublish()
	{
		// Init delay
		hrt_abstime current_time = hrt_absolute_time();

		if (current_time < delay_us) {
			return false;
		}

		return true;
	}

private:
	uint16_t _id{0};
	hrt_abstime delay_us = 0;
};
} // namespace uavcannode
